// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
@TestOn('!chrome')
library;

import 'dart:ui' as ui show BoxHeightStyle, BoxWidthStyle;
import 'dart:ui';

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import '../impeller_test_helpers.dart';
import '../widgets/clipboard_utils.dart';
import '../widgets/editable_text_utils.dart' show textOffsetToPosition;
import '../widgets/semantics_tester.dart';

class MaterialLocalizationsDelegate extends LocalizationsDelegate<MaterialLocalizations> {
  @override
  bool isSupported(Locale locale) => true;

  @override
  Future<MaterialLocalizations> load(Locale locale) => DefaultMaterialLocalizations.load(locale);

  @override
  bool shouldReload(MaterialLocalizationsDelegate old) => false;
}

class WidgetsLocalizationsDelegate extends LocalizationsDelegate<WidgetsLocalizations> {
  @override
  bool isSupported(Locale locale) => true;

  @override
  Future<WidgetsLocalizations> load(Locale locale) => DefaultWidgetsLocalizations.load(locale);

  @override
  bool shouldReload(WidgetsLocalizationsDelegate old) => false;
}

Widget overlay({Widget? child}) {
  final OverlayEntry entry = OverlayEntry(
    builder: (BuildContext context) {
      return Center(child: Material(child: child));
    },
  );
  addTearDown(
    () =>
        entry
          ..remove()
          ..dispose(),
  );
  return overlayWithEntry(entry);
}

Widget overlayWithEntry(OverlayEntry entry) {
  return Theme(
    data: ThemeData(useMaterial3: false),
    child: Localizations(
      locale: const Locale('en', 'US'),
      delegates: <LocalizationsDelegate<dynamic>>[
        WidgetsLocalizationsDelegate(),
        MaterialLocalizationsDelegate(),
      ],
      child: Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: const MediaQueryData(size: Size(800.0, 600.0)),
          child: Overlay(initialEntries: <OverlayEntry>[entry]),
        ),
      ),
    ),
  );
}

Widget boilerplate({Widget? child}) {
  return Theme(
    data: ThemeData(useMaterial3: false),
    child: Localizations(
      locale: const Locale('en', 'US'),
      delegates: <LocalizationsDelegate<dynamic>>[
        WidgetsLocalizationsDelegate(),
        MaterialLocalizationsDelegate(),
      ],
      child: Directionality(
        textDirection: TextDirection.ltr,
        child: MediaQuery(
          data: const MediaQueryData(size: Size(800.0, 600.0)),
          child: Center(child: Material(child: child)),
        ),
      ),
    ),
  );
}

Future<void> skipPastScrollingAnimation(WidgetTester tester) async {
  await tester.pump();
  await tester.pump(const Duration(milliseconds: 200));
}

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();
  final MockClipboard mockClipboard = MockClipboard();

  const String kThreeLines =
      'First line of text is\n'
      'Second line goes until\n'
      'Third line of stuff';
  const String kMoreThanFourLines =
      '$kThreeLines\n'
      "Fourth line won't display and ends at";

  // Returns the first RenderEditable.
  RenderEditable findRenderEditable(WidgetTester tester) {
    final RenderObject root = tester.renderObject(find.byType(EditableText));
    expect(root, isNotNull);

    late RenderEditable renderEditable;
    void recursiveFinder(RenderObject child) {
      if (child is RenderEditable) {
        renderEditable = child;
        return;
      }
      child.visitChildren(recursiveFinder);
    }

    root.visitChildren(recursiveFinder);
    expect(renderEditable, isNotNull);
    return renderEditable;
  }

  // Check that the Cupertino text selection toolbar is the expected one on iOS and macOS.
  // TODO(bleroux): try to merge this into text_selection_toolbar_utils.dart
  //                (for instance by adding a 'readOnly' flag).
  void expectCupertinoSelectionToolbar() {
    // This function is valid only for tests running on Apple platforms.
    expect(
      defaultTargetPlatform == TargetPlatform.iOS || defaultTargetPlatform == TargetPlatform.macOS,
      isTrue,
    );

    if (defaultTargetPlatform == TargetPlatform.iOS) {
      expect(find.byType(CupertinoButton), findsNWidgets(4));
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Look Up'), findsOneWidget);
      expect(find.text('Search Web'), findsOneWidget);
      expect(find.text('Share...'), findsOneWidget);
    } else {
      expect(find.byType(CupertinoButton), findsNWidgets(1));
      expect(find.text('Copy'), findsOneWidget);
    }
  }

  // Check that the Material text selection toolbar is the expected one.
  // TODO(bleroux): Try to merge this into text_selection_toolbar_utils.dart
  //                (for instance by adding a 'readOnly' flag).
  void expectMaterialSelectionToolbar() {
    if (defaultTargetPlatform == TargetPlatform.android) {
      expect(find.byType(TextButton), findsNWidgets(3));
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Share'), findsOneWidget);
      expect(find.text('Select all'), findsOneWidget);
    } else {
      expect(find.byType(TextButton), findsNWidgets(2));
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Select all'), findsOneWidget);
    }
  }

  List<TextSelectionPoint> globalize(Iterable<TextSelectionPoint> points, RenderBox box) {
    return points.map<TextSelectionPoint>((TextSelectionPoint point) {
      return TextSelectionPoint(box.localToGlobal(point.point), point.direction);
    }).toList();
  }

  setUp(() async {
    debugResetSemanticsIdCounter();
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      mockClipboard.handleMethodCall,
    );
    // Fill the clipboard so that the Paste option is available in the text
    // selection menu.
    await Clipboard.setData(const ClipboardData(text: 'Clipboard data'));
  });

  tearDown(() {
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      null,
    );
  });

  Widget selectableTextBuilder({String text = '', int? maxLines = 1, int? minLines}) {
    return boilerplate(
      child: SelectableText(
        text,
        style: const TextStyle(color: Colors.black, fontSize: 34.0),
        maxLines: maxLines,
        minLines: minLines,
      ),
    );
  }

  testWidgets(
    'throw if no Overlay widget exists above',
    experimentalLeakTesting:
        LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const Directionality(
          textDirection: TextDirection.ltr,
          child: MediaQuery(
            data: MediaQueryData(size: Size(800.0, 600.0)),
            child: Center(child: Material(child: SelectableText('I love Flutter!'))),
          ),
        ),
      );

      final Offset textFieldStart = tester.getTopLeft(find.byType(SelectableText));
      final TestGesture gesture = await tester.startGesture(
        textFieldStart,
        kind: PointerDeviceKind.mouse,
      );
      await tester.pump(const Duration(seconds: 2));
      await gesture.up();
      await tester.pumpAndSettle(kDoubleTapTimeout);

      final FlutterError error = tester.takeException() as FlutterError;
      expect(error.message, contains('EditableText widgets require an Overlay widget ancestor'));

      await tester.pumpWidget(const SizedBox.shrink());
      expect(tester.takeException(), isNotNull); // side effect exception
    },
  );

  testWidgets('Do not crash when remove SelectableText during handle drag', (
    WidgetTester tester,
  ) async {
    // Regression test https://github.com/flutter/flutter/issues/108242
    bool isShow = true;
    late StateSetter setter;
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              setter = setState;
              if (isShow) {
                return const SelectableText(
                  'abc def ghi',
                  dragStartBehavior: DragStartBehavior.down,
                );
              } else {
                return const SizedBox.shrink();
              }
            },
          ),
        ),
      ),
    );

    final EditableText editableTextWidget = tester.widget(find.byType(EditableText));
    final TextEditingController controller = editableTextWidget.controller;

    // Long press the 'e' to select 'def'.
    final Offset ePos = textOffsetToPosition(tester, 5);
    TestGesture gesture = await tester.startGesture(ePos, pointer: 7);
    await tester.pump(const Duration(seconds: 2));
    await gesture.up();
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 200),
    ); // skip past the frame where the opacity is zero

    final TextSelection selection = controller.selection;
    expect(selection.baseOffset, 4);
    expect(selection.extentOffset, 7);

    final RenderEditable renderEditable = findRenderEditable(tester);
    final List<TextSelectionPoint> endpoints = globalize(
      renderEditable.getEndpointsForSelection(selection),
      renderEditable,
    );
    expect(endpoints.length, 2);

    // Drag the left handle to the left.
    final Offset handlePos = endpoints[0].point + const Offset(-1.0, 1.0);
    final Offset newHandlePos = textOffsetToPosition(tester, 1);
    final Offset newHandlePos1 = textOffsetToPosition(tester, 0);
    gesture = await tester.startGesture(handlePos, pointer: 7);
    await tester.pump();
    await gesture.moveTo(newHandlePos);
    await tester.pump();

    // Unmount the SelectableText during handle drag.
    setter(() {
      isShow = false;
    });
    await tester.pump();

    await gesture.moveTo(newHandlePos1);
    await tester.pump(); // Do not crash here.

    await gesture.up();
    await tester.pump();
  });

  testWidgets('has expected defaults', (WidgetTester tester) async {
    await tester.pumpWidget(boilerplate(child: const SelectableText('selectable text')));

    final SelectableText selectableText = tester.firstWidget(find.byType(SelectableText));
    expect(selectableText.showCursor, false);
    expect(selectableText.autofocus, false);
    expect(selectableText.dragStartBehavior, DragStartBehavior.start);
    expect(selectableText.cursorWidth, 2.0);
    expect(selectableText.cursorHeight, isNull);
    expect(selectableText.enableInteractiveSelection, true);
  });

  testWidgets('Rich selectable text has expected defaults', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MediaQuery(
        data: MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: SelectableText.rich(
            TextSpan(
              text: 'First line!',
              style: TextStyle(fontSize: 14, fontFamily: 'Roboto'),
              children: <TextSpan>[
                TextSpan(
                  text: 'Second line!\n',
                  style: TextStyle(fontSize: 30, fontFamily: 'Roboto'),
                ),
                TextSpan(
                  text: 'Third line!\n',
                  style: TextStyle(fontSize: 14, fontFamily: 'Roboto'),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    final SelectableText selectableText = tester.firstWidget(find.byType(SelectableText));
    expect(selectableText.showCursor, false);
    expect(selectableText.autofocus, false);
    expect(selectableText.dragStartBehavior, DragStartBehavior.start);
    expect(selectableText.cursorWidth, 2.0);
    expect(selectableText.cursorHeight, isNull);
    expect(selectableText.enableInteractiveSelection, true);
  });

  testWidgets('Rich selectable text supports WidgetSpan', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MediaQuery(
        data: MediaQueryData(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: SelectableText.rich(
            TextSpan(
              text: 'First line!',
              style: TextStyle(fontSize: 14, fontFamily: 'Roboto'),
              children: <InlineSpan>[
                WidgetSpan(
                  child: SizedBox(
                    width: 120,
                    height: 50,
                    child: Card(child: Center(child: Text('Hello World!'))),
                  ),
                ),
                TextSpan(
                  text: 'Third line!\n',
                  style: TextStyle(fontSize: 14, fontFamily: 'Roboto'),
                ),
              ],
            ),
          ),
        ),
      ),
    );
    expect(tester.takeException(), isNull);
  });

  testWidgets('no text keyboard when widget is focused', (WidgetTester tester) async {
    await tester.pumpWidget(overlay(child: const SelectableText('selectable text')));
    await tester.tap(find.byType(SelectableText));
    await tester.idle();
    expect(tester.testTextInput.hasAnyClients, false);
  });

  testWidgets('uses DefaultSelectionStyle for selection and cursor colors if provided', (
    WidgetTester tester,
  ) async {
    const Color selectionColor = Colors.orange;
    const Color cursorColor = Colors.red;

    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: DefaultSelectionStyle(
            selectionColor: selectionColor,
            cursorColor: cursorColor,
            child: SelectableText('text'),
          ),
        ),
      ),
    );
    await tester.pump();
    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    expect(state.widget.selectionColor, selectionColor);
    expect(state.widget.cursorColor, cursorColor);
  });

  testWidgets('Selectable Text has adaptive size', (WidgetTester tester) async {
    await tester.pumpWidget(boilerplate(child: const SelectableText('s')));

    RenderBox findSelectableTextBox() => tester.renderObject(find.byType(SelectableText));

    final RenderBox textBox = findSelectableTextBox();
    expect(textBox.size, const Size(17.0, 14.0));

    await tester.pumpWidget(boilerplate(child: const SelectableText('very very long')));

    final RenderBox longtextBox = findSelectableTextBox();
    expect(longtextBox.size, const Size(199.0, 14.0));
  });

  testWidgets('can scale with textScaleFactor', (WidgetTester tester) async {
    await tester.pumpWidget(boilerplate(child: const SelectableText('selectable text')));

    final RenderBox renderBox = tester.renderObject(find.byType(SelectableText));
    expect(renderBox.size.height, 14.0);

    await tester.pumpWidget(
      boilerplate(child: const SelectableText('selectable text', textScaleFactor: 1.9)),
    );

    final RenderBox scaledBox = tester.renderObject(find.byType(SelectableText));
    expect(scaledBox.size.height, 27.0);
  });

  testWidgets('can switch between textWidthBasis', (WidgetTester tester) async {
    RenderBox findTextBox() => tester.renderObject(find.byType(SelectableText));
    const String text = 'I can face roll keyboardkeyboardaszzaaaaszzaaaaszzaaaaszzaaaa';
    await tester.pumpWidget(
      boilerplate(child: const SelectableText(text, textWidthBasis: TextWidthBasis.parent)),
    );
    RenderBox textBox = findTextBox();
    expect(textBox.size, const Size(800.0, 28.0));

    await tester.pumpWidget(
      boilerplate(child: const SelectableText(text, textWidthBasis: TextWidthBasis.longestLine)),
    );
    textBox = findTextBox();
    expect(textBox.size, const Size(633.0, 28.0));
  });

  testWidgets('can switch between textHeightBehavior', (WidgetTester tester) async {
    const String text = 'selectable text';
    const TextHeightBehavior textHeightBehavior = TextHeightBehavior(
      applyHeightToFirstAscent: false,
      applyHeightToLastDescent: false,
    );
    await tester.pumpWidget(boilerplate(child: const SelectableText(text)));
    expect(findRenderEditable(tester).textHeightBehavior, isNull);

    await tester.pumpWidget(
      boilerplate(child: const SelectableText(text, textHeightBehavior: textHeightBehavior)),
    );
    expect(findRenderEditable(tester).textHeightBehavior, textHeightBehavior);
  });

  testWidgets('Cursor blinks when showCursor is true', (WidgetTester tester) async {
    await tester.pumpWidget(overlay(child: const SelectableText('some text', showCursor: true)));
    await tester.tap(find.byType(SelectableText));
    await tester.idle();

    final EditableTextState editableText = tester.state(find.byType(EditableText));

    // Check that the cursor visibility toggles after each blink interval.
    final bool initialShowCursor = editableText.cursorCurrentlyVisible;
    await tester.pump(editableText.cursorBlinkInterval);
    expect(editableText.cursorCurrentlyVisible, equals(!initialShowCursor));
    await tester.pump(editableText.cursorBlinkInterval);
    expect(editableText.cursorCurrentlyVisible, equals(initialShowCursor));
    await tester.pump(editableText.cursorBlinkInterval ~/ 10);
    expect(editableText.cursorCurrentlyVisible, equals(initialShowCursor));
    await tester.pump(editableText.cursorBlinkInterval);
    expect(editableText.cursorCurrentlyVisible, equals(!initialShowCursor));
    await tester.pump(editableText.cursorBlinkInterval);
    expect(editableText.cursorCurrentlyVisible, equals(initialShowCursor));
  });

  testWidgets('selectable text selection toolbar renders correctly inside opacity', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: Center(
            child: SizedBox(
              width: 100,
              height: 100,
              child: Opacity(opacity: 0.5, child: SelectableText('selectable text')),
            ),
          ),
        ),
      ),
    );

    // The selectWordsInRange with SelectionChangedCause.tap seems to be needed to show the toolbar.
    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    state.renderEditable.selectWordsInRange(from: Offset.zero, cause: SelectionChangedCause.tap);

    expect(state.showToolbar(), true);

    // This is needed for the AnimatedOpacity to turn from 0 to 1 so the toolbar is visible.
    await tester.pumpAndSettle();
    await tester.pump(const Duration(seconds: 1));

    expect(find.text('Select all'), findsOneWidget);
  });

  testWidgets('Caret position is updated on tap', (WidgetTester tester) async {
    await tester.pumpWidget(overlay(child: const SelectableText('abc def ghi')));
    final EditableText editableText = tester.widget(find.byType(EditableText));
    expect(editableText.controller.selection.baseOffset, -1);
    expect(editableText.controller.selection.extentOffset, -1);

    // Tap to reposition the caret.
    const int tapIndex = 4;
    final Offset ePos = textOffsetToPosition(tester, tapIndex);
    await tester.tapAt(ePos);
    await tester.pump();

    expect(editableText.controller.selection.baseOffset, tapIndex);
    expect(editableText.controller.selection.extentOffset, tapIndex);
  });

  testWidgets('enableInteractiveSelection = false, tap', (WidgetTester tester) async {
    await tester.pumpWidget(
      overlay(child: const SelectableText('abc def ghi', enableInteractiveSelection: false)),
    );
    final EditableText editableText = tester.widget(find.byType(EditableText));
    expect(editableText.controller.selection.baseOffset, -1);
    expect(editableText.controller.selection.extentOffset, -1);

    // Tap would ordinarily reposition the caret.
    const int tapIndex = 4;
    final Offset ePos = textOffsetToPosition(tester, tapIndex);
    await tester.tapAt(ePos);
    await tester.pump();

    expect(editableText.controller.selection.baseOffset, -1);
    expect(editableText.controller.selection.extentOffset, -1);
  });

  testWidgets('enableInteractiveSelection = false, long-press', (WidgetTester tester) async {
    await tester.pumpWidget(
      overlay(child: const SelectableText('abc def ghi', enableInteractiveSelection: false)),
    );
    final EditableText editableText = tester.widget(find.byType(EditableText));
    expect(editableText.controller.selection.baseOffset, -1);
    expect(editableText.controller.selection.extentOffset, -1);

    // Long press the 'e' to select 'def'.
    final Offset ePos = textOffsetToPosition(tester, 5);
    final TestGesture gesture = await tester.startGesture(ePos, pointer: 7);
    await tester.pump(const Duration(seconds: 2));
    await gesture.up();
    await tester.pump();

    expect(editableText.controller.selection.isCollapsed, true);
    expect(editableText.controller.selection.baseOffset, -1);
    expect(editableText.controller.selection.extentOffset, -1);
  });

  testWidgets('Can long press to select', (WidgetTester tester) async {
    await tester.pumpWidget(overlay(child: const SelectableText('abc def ghi')));

    final EditableText editableText = tester.widget(find.byType(EditableText));

    expect(editableText.controller.selection.isCollapsed, true);

    // Long press the 'e' to select 'def'.
    const int tapIndex = 5;
    final Offset ePos = textOffsetToPosition(tester, tapIndex);
    await tester.longPressAt(ePos);
    await tester.pump();

    // 'def' is selected.
    expect(editableText.controller.selection.baseOffset, 4);
    expect(editableText.controller.selection.extentOffset, 7);

    // Tapping elsewhere immediately collapses and moves the cursor.
    await tester.tapAt(textOffsetToPosition(tester, 9));
    await tester.pump();

    expect(editableText.controller.selection.isCollapsed, true);
    expect(editableText.controller.selection.baseOffset, 9);
  });

  testWidgets("Slight movements in longpress don't hide/show handles", (WidgetTester tester) async {
    await tester.pumpWidget(overlay(child: const SelectableText('abc def ghi')));
    // Long press the 'e' to select 'def', but don't release the gesture.
    final Offset ePos = textOffsetToPosition(tester, 5);
    final TestGesture gesture = await tester.startGesture(ePos, pointer: 7);
    await tester.pump(const Duration(seconds: 2));
    await tester.pumpAndSettle();

    // Handles are shown.
    final Finder fadeFinder = find.byType(FadeTransition);
    expect(fadeFinder, findsNWidgets(2)); // 2 handles, 1 toolbar
    FadeTransition handle = tester.widget(fadeFinder.at(0));
    expect(handle.opacity.value, equals(1.0));

    // Move the gesture very slightly.
    await gesture.moveBy(const Offset(1.0, 1.0));
    await tester.pump(SelectionOverlay.fadeDuration * 0.5);
    handle = tester.widget(fadeFinder.at(0));

    // The handle should still be fully opaque.
    expect(handle.opacity.value, equals(1.0));
  });

  testWidgets('Mouse long press is just like a tap', (WidgetTester tester) async {
    await tester.pumpWidget(overlay(child: const SelectableText('abc def ghi')));

    final EditableText editableText = tester.widget(find.byType(EditableText));

    // Long press the 'e' using a mouse device.
    const int eIndex = 5;
    final Offset ePos = textOffsetToPosition(tester, eIndex);
    final TestGesture gesture = await tester.startGesture(ePos, kind: PointerDeviceKind.mouse);
    await tester.pump(const Duration(seconds: 2));
    await gesture.up();
    await tester.pump();

    // The cursor is placed just like a regular tap.
    expect(editableText.controller.selection.baseOffset, eIndex);
    expect(editableText.controller.selection.extentOffset, eIndex);
  });

  testWidgets('selectable text basic', (WidgetTester tester) async {
    await tester.pumpWidget(overlay(child: const SelectableText('selectable')));
    final EditableText editableTextWidget = tester.widget(find.byType(EditableText));
    // Selectable text cannot open keyboard.
    await tester.showKeyboard(find.byType(SelectableText));
    expect(tester.testTextInput.hasAnyClients, false);
    await skipPastScrollingAnimation(tester);

    expect(editableTextWidget.controller.selection.isCollapsed, true);

    await tester.tap(find.byType(SelectableText));
    await tester.pump();

    final EditableTextState editableText = tester.state(find.byType(EditableText));
    // Collapse selection should not paint.
    expect(editableText.selectionOverlay!.handlesAreVisible, isFalse);
    // Long press on the 't' character of text 'selectable' to show context menu.
    const int dIndex = 5;
    final Offset dPos = textOffsetToPosition(tester, dIndex);
    await tester.longPressAt(dPos);
    await tester.pump();

    // Context menu should not have paste and cut.
    expect(find.text('Copy'), findsOneWidget);
    expect(find.text('Paste'), findsNothing);
    expect(find.text('Cut'), findsNothing);
  });

  testWidgets('selectable text can disable toolbar options', (WidgetTester tester) async {
    await tester.pumpWidget(
      overlay(
        child: const SelectableText(
          'a selectable text',
          toolbarOptions: ToolbarOptions(selectAll: true),
        ),
      ),
    );
    const int dIndex = 5;
    final Offset dPos = textOffsetToPosition(tester, dIndex);
    await tester.longPressAt(dPos);
    await tester.pump();
    // Context menu should not have copy.
    expect(find.text('Copy'), findsNothing);
    expect(find.text('Select all'), findsOneWidget);
  });

  testWidgets('Can select text by dragging with a mouse', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: SelectableText('abc def ghi', dragStartBehavior: DragStartBehavior.down),
        ),
      ),
    );
    final EditableText editableTextWidget = tester.widget(find.byType(EditableText));
    final TextEditingController controller = editableTextWidget.controller;

    final Offset ePos = textOffsetToPosition(tester, 5);
    final Offset gPos = textOffsetToPosition(tester, 8);

    final TestGesture gesture = await tester.startGesture(ePos, kind: PointerDeviceKind.mouse);
    await tester.pump();
    await gesture.moveTo(gPos);
    await tester.pump();
    await gesture.up();
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, 5);
    expect(controller.selection.extentOffset, 8);
  });

  testWidgets('Continuous dragging does not cause flickering', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: SelectableText(
            'abc def ghi',
            dragStartBehavior: DragStartBehavior.down,
            style: TextStyle(fontSize: 10.0),
          ),
        ),
      ),
    );
    final EditableText editableTextWidget = tester.widget(find.byType(EditableText));
    final TextEditingController controller = editableTextWidget.controller;

    int selectionChangedCount = 0;

    controller.addListener(() {
      selectionChangedCount++;
    });

    final Offset cPos = textOffsetToPosition(tester, 2); // Index of 'c'.
    final Offset gPos = textOffsetToPosition(tester, 8); // Index of 'g'.
    final Offset hPos = textOffsetToPosition(tester, 9); // Index of 'h'.

    // Drag from 'c' to 'g'.
    final TestGesture gesture = await tester.startGesture(cPos, kind: PointerDeviceKind.mouse);
    await tester.pump();
    await gesture.moveTo(gPos);
    await tester.pumpAndSettle();

    expect(selectionChangedCount, isNonZero);
    selectionChangedCount = 0;
    expect(controller.selection.baseOffset, 2);
    expect(controller.selection.extentOffset, 8);

    // Tiny movement shouldn't cause text selection to change.
    await gesture.moveTo(gPos + const Offset(4.0, 0.0));
    await tester.pumpAndSettle();
    expect(selectionChangedCount, 0);

    // Now a text selection change will occur after a significant movement.
    await gesture.moveTo(hPos);
    await tester.pump();
    await gesture.up();
    await tester.pumpAndSettle();

    expect(selectionChangedCount, 1);
    expect(controller.selection.baseOffset, 2);
    expect(controller.selection.extentOffset, 9);
  });

  testWidgets('Dragging in opposite direction also works', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: SelectableText('abc def ghi', dragStartBehavior: DragStartBehavior.down),
        ),
      ),
    );
    final EditableText editableTextWidget = tester.widget(find.byType(EditableText));
    final TextEditingController controller = editableTextWidget.controller;

    final Offset ePos = textOffsetToPosition(tester, 5);
    final Offset gPos = textOffsetToPosition(tester, 8);

    final TestGesture gesture = await tester.startGesture(gPos, kind: PointerDeviceKind.mouse);
    await tester.pump();
    await gesture.moveTo(ePos);
    await tester.pump();
    await gesture.up();
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, 8);
    expect(controller.selection.extentOffset, 5);
  });

  testWidgets('Slow mouse dragging also selects text', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: SelectableText('abc def ghi', dragStartBehavior: DragStartBehavior.down),
        ),
      ),
    );
    final EditableText editableTextWidget = tester.widget(find.byType(EditableText));
    final TextEditingController controller = editableTextWidget.controller;

    final Offset ePos = textOffsetToPosition(tester, 5);
    final Offset gPos = textOffsetToPosition(tester, 8);

    final TestGesture gesture = await tester.startGesture(ePos, kind: PointerDeviceKind.mouse);
    await tester.pump(const Duration(seconds: 2));
    await gesture.moveTo(gPos);
    await tester.pump();
    await gesture.up();

    expect(controller.selection.baseOffset, 5);
    expect(controller.selection.extentOffset, 8);
  });

  testWidgets('Can drag handles to change selection', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: SelectableText('abc def ghi', dragStartBehavior: DragStartBehavior.down),
        ),
      ),
    );
    final EditableText editableTextWidget = tester.widget(find.byType(EditableText));
    final TextEditingController controller = editableTextWidget.controller;

    // Long press the 'e' to select 'def'.
    final Offset ePos = textOffsetToPosition(tester, 5);
    TestGesture gesture = await tester.startGesture(ePos, pointer: 7);
    await tester.pump(const Duration(seconds: 2));
    await gesture.up();
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 200),
    ); // skip past the frame where the opacity is zero

    final TextSelection selection = controller.selection;
    expect(selection.baseOffset, 4);
    expect(selection.extentOffset, 7);

    final RenderEditable renderEditable = findRenderEditable(tester);
    final List<TextSelectionPoint> endpoints = globalize(
      renderEditable.getEndpointsForSelection(selection),
      renderEditable,
    );
    expect(endpoints.length, 2);

    // Drag the right handle 2 letters to the right.
    // We use a small offset because the endpoint is on the very corner
    // of the handle.
    Offset handlePos = endpoints[1].point + const Offset(1.0, 1.0);
    Offset newHandlePos = textOffsetToPosition(tester, 11);
    gesture = await tester.startGesture(handlePos, pointer: 7);
    await tester.pump();
    await gesture.moveTo(newHandlePos);
    await tester.pump();
    await gesture.up();
    await tester.pump();

    expect(controller.selection.baseOffset, 4);
    expect(controller.selection.extentOffset, 11);

    // Drag the left handle 2 letters to the left.
    handlePos = endpoints[0].point + const Offset(-1.0, 1.0);
    newHandlePos = textOffsetToPosition(tester, 0);
    gesture = await tester.startGesture(handlePos, pointer: 7);
    await tester.pump();
    await gesture.moveTo(newHandlePos);
    await tester.pump();
    await gesture.up();
    await tester.pump();

    expect(controller.selection.baseOffset, 0);
    expect(controller.selection.extentOffset, 11);
  });

  testWidgets('Dragging handles calls onSelectionChanged', (WidgetTester tester) async {
    TextSelection? newSelection;
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SelectableText(
            'abc def ghi',
            dragStartBehavior: DragStartBehavior.down,
            onSelectionChanged: (TextSelection selection, SelectionChangedCause? cause) {
              expect(newSelection, isNull);
              newSelection = selection;
            },
          ),
        ),
      ),
    );

    // Long press the 'e' to select 'def'.
    final Offset ePos = textOffsetToPosition(tester, 5);
    TestGesture gesture = await tester.startGesture(ePos, pointer: 7);
    await tester.pump(const Duration(seconds: 2));
    await gesture.up();
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 200),
    ); // skip past the frame where the opacity is zero

    expect(newSelection!.baseOffset, 4);
    expect(newSelection!.extentOffset, 7);

    final RenderEditable renderEditable = findRenderEditable(tester);
    final List<TextSelectionPoint> endpoints = globalize(
      renderEditable.getEndpointsForSelection(newSelection!),
      renderEditable,
    );
    expect(endpoints.length, 2);
    newSelection = null;

    // Drag the right handle 2 letters to the right.
    // We use a small offset because the endpoint is on the very corner
    // of the handle.
    final Offset handlePos = endpoints[1].point + const Offset(1.0, 1.0);
    final Offset newHandlePos = textOffsetToPosition(tester, 9);
    gesture = await tester.startGesture(handlePos, pointer: 7);
    await tester.pump();
    await gesture.moveTo(newHandlePos);
    await tester.pump();
    await gesture.up();
    await tester.pump();

    expect(newSelection!.baseOffset, 4);
    expect(newSelection!.extentOffset, 9);
  });

  testWidgets('Cannot drag one handle past the other', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: SelectableText('abc def ghi', dragStartBehavior: DragStartBehavior.down),
        ),
      ),
    );
    final EditableText editableTextWidget = tester.widget(find.byType(EditableText));
    final TextEditingController controller = editableTextWidget.controller;

    // Long press the 'e' to select 'def'.
    final Offset ePos = textOffsetToPosition(tester, 5); // Position before 'e'.
    TestGesture gesture = await tester.startGesture(ePos, pointer: 7);
    await tester.pump(const Duration(seconds: 2));
    await gesture.up();
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 200),
    ); // skip past the frame where the opacity is zero

    final TextSelection selection = controller.selection;
    expect(selection.baseOffset, 4);
    expect(selection.extentOffset, 7);

    final RenderEditable renderEditable = findRenderEditable(tester);
    final List<TextSelectionPoint> endpoints = globalize(
      renderEditable.getEndpointsForSelection(selection),
      renderEditable,
    );
    expect(endpoints.length, 2);

    // Drag the right handle until there's only 1 char selected.
    // We use a small offset because the endpoint is on the very corner
    // of the handle.
    final Offset handlePos = endpoints[1].point + const Offset(4.0, 0.0);
    Offset newHandlePos = textOffsetToPosition(tester, 5); // Position before 'e'.
    gesture = await tester.startGesture(handlePos, pointer: 7);
    await tester.pump();
    await gesture.moveTo(newHandlePos);
    await tester.pump();

    expect(controller.selection.baseOffset, 4);
    expect(controller.selection.extentOffset, 5);

    newHandlePos = textOffsetToPosition(tester, 2); // Position before 'c'.
    await gesture.moveTo(newHandlePos);
    await tester.pump();
    await gesture.up();
    await tester.pump();

    expect(controller.selection.baseOffset, 4);
    // The selection doesn't move beyond the left handle. There's always at
    // least 1 char selected.
    expect(controller.selection.extentOffset, 5);
  });

  testWidgets('Can use selection toolbar', (WidgetTester tester) async {
    const String testValue = 'abc def ghi';
    await tester.pumpWidget(const MaterialApp(home: Material(child: SelectableText(testValue))));
    final EditableText editableTextWidget = tester.widget(find.byType(EditableText));
    final TextEditingController controller = editableTextWidget.controller;

    // Tap the selection handle to bring up the "paste / select all" menu.
    await tester.tapAt(textOffsetToPosition(tester, testValue.indexOf('e')));
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 200),
    ); // skip past the frame where the opacity is zero
    final RenderEditable renderEditable = findRenderEditable(tester);
    final List<TextSelectionPoint> endpoints = globalize(
      renderEditable.getEndpointsForSelection(controller.selection),
      renderEditable,
    );
    // Tapping on the part of the handle's GestureDetector where it overlaps
    // with the text itself does not show the menu, so add a small vertical
    // offset to tap below the text.
    await tester.tapAt(endpoints[0].point + const Offset(1.0, 13.0));
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 200),
    ); // skip past the frame where the opacity is zero

    // Select all should select all the text.
    await tester.tap(find.text('Select all'));
    await tester.pump();
    expect(controller.selection.baseOffset, 0);
    expect(controller.selection.extentOffset, testValue.length);

    // Copy should reset the selection.
    await tester.tap(find.text('Copy'));
    await skipPastScrollingAnimation(tester);
    expect(controller.selection.isCollapsed, true);
  });

  testWidgets('Selectable height with maxLine', (WidgetTester tester) async {
    await tester.pumpWidget(selectableTextBuilder());

    RenderBox findTextBox() => tester.renderObject(find.byType(SelectableText));

    final RenderBox textBox = findTextBox();
    final Size emptyInputSize = textBox.size;

    await tester.pumpWidget(selectableTextBuilder(text: 'No wrapping here.'));
    expect(findTextBox(), equals(textBox));
    expect(textBox.size.height, emptyInputSize.height);

    // Even when entering multiline text, SelectableText doesn't grow. It's a single
    // line input.
    await tester.pumpWidget(selectableTextBuilder(text: kThreeLines));
    expect(findTextBox(), equals(textBox));
    expect(textBox.size.height, emptyInputSize.height);

    // maxLines: 3 makes the SelectableText 3 lines tall.
    await tester.pumpWidget(selectableTextBuilder(maxLines: 3));
    expect(findTextBox(), equals(textBox));
    expect(textBox.size.height, greaterThan(emptyInputSize.height));

    final Size threeLineInputSize = textBox.size;

    // Filling with 3 lines of text stays the same size.
    await tester.pumpWidget(selectableTextBuilder(text: kThreeLines, maxLines: 3));
    expect(findTextBox(), equals(textBox));
    expect(textBox.size.height, threeLineInputSize.height);

    // An extra line won't increase the size because we max at 3.
    await tester.pumpWidget(selectableTextBuilder(text: kMoreThanFourLines, maxLines: 3));
    expect(findTextBox(), equals(textBox));
    expect(textBox.size.height, threeLineInputSize.height);

    // But now it will... but it will max at four.
    await tester.pumpWidget(selectableTextBuilder(text: kMoreThanFourLines, maxLines: 4));
    expect(findTextBox(), equals(textBox));
    expect(textBox.size.height, greaterThan(threeLineInputSize.height));

    final Size fourLineInputSize = textBox.size;

    // Now it won't max out until the end.
    await tester.pumpWidget(selectableTextBuilder(maxLines: null));
    expect(findTextBox(), equals(textBox));
    expect(textBox.size, equals(emptyInputSize));
    await tester.pumpWidget(selectableTextBuilder(text: kThreeLines, maxLines: null));
    expect(textBox.size.height, equals(threeLineInputSize.height));
    await tester.pumpWidget(selectableTextBuilder(text: kMoreThanFourLines, maxLines: null));
    expect(textBox.size.height, greaterThan(fourLineInputSize.height));
  });

  testWidgets('Can drag handles to change selection in multiline', (WidgetTester tester) async {
    const String testValue = kThreeLines;
    await tester.pumpWidget(
      overlay(
        child: const SelectableText(
          testValue,
          dragStartBehavior: DragStartBehavior.down,
          style: TextStyle(color: Colors.black, fontSize: 34.0),
          maxLines: 3,
        ),
      ),
    );

    final EditableText editableTextWidget = tester.widget(find.byType(EditableText));
    final TextEditingController controller = editableTextWidget.controller;

    // Check that the text spans multiple lines.
    final Offset firstPos = textOffsetToPosition(tester, testValue.indexOf('First'));
    final Offset secondPos = textOffsetToPosition(tester, testValue.indexOf('Second'));
    final Offset thirdPos = textOffsetToPosition(tester, testValue.indexOf('Third'));
    final Offset middleStringPos = textOffsetToPosition(tester, testValue.indexOf('irst'));

    expect(firstPos.dx, 24.5);
    expect(secondPos.dx, 24.5);
    expect(thirdPos.dx, 24.5);
    expect(middleStringPos.dx, 58.5);
    expect(firstPos.dx, secondPos.dx);
    expect(firstPos.dx, thirdPos.dx);
    expect(firstPos.dy, lessThan(secondPos.dy));
    expect(secondPos.dy, lessThan(thirdPos.dy));

    // Long press the 'n' in 'until' to select the word.
    final Offset untilPos = textOffsetToPosition(tester, testValue.indexOf('until') + 1);
    TestGesture gesture = await tester.startGesture(untilPos, pointer: 7);
    await tester.pump(const Duration(seconds: 2));
    await gesture.up();
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 200),
    ); // skip past the frame where the opacity is zero

    expect(controller.selection.baseOffset, 39);
    expect(controller.selection.extentOffset, 44);

    final RenderEditable renderEditable = findRenderEditable(tester);
    final List<TextSelectionPoint> endpoints = globalize(
      renderEditable.getEndpointsForSelection(controller.selection),
      renderEditable,
    );
    expect(endpoints.length, 2);

    // Drag the right handle to the third line, just after 'Third'.
    Offset handlePos = endpoints[1].point + const Offset(1.0, 1.0);
    // The distance below the y value returned by textOffsetToPosition required
    // to register a full vertical line drag.
    const Offset downLineOffset = Offset(0.0, 3.0);
    Offset newHandlePos =
        textOffsetToPosition(tester, testValue.indexOf('Third') + 5) + downLineOffset;
    gesture = await tester.startGesture(handlePos, pointer: 7);
    await tester.pump();
    await gesture.moveTo(newHandlePos);
    await tester.pump();
    await gesture.up();
    await tester.pump();

    expect(controller.selection, const TextSelection(baseOffset: 39, extentOffset: 50));

    // Drag the left handle to the first line, just after 'First'.
    handlePos = endpoints[0].point + const Offset(-1.0, 1.0);
    newHandlePos = textOffsetToPosition(tester, testValue.indexOf('First') + 5);
    gesture = await tester.startGesture(handlePos, pointer: 7);
    await tester.pump();
    await gesture.moveTo(newHandlePos);
    await tester.pump();
    await gesture.up();
    await tester.pump();

    expect(controller.selection.baseOffset, 5);
    expect(controller.selection.extentOffset, 50);
    await tester.tap(find.text('Copy'));
    await tester.pump();
    expect(controller.selection.isCollapsed, true);
  });

  testWidgets('Can scroll multiline input', (WidgetTester tester) async {
    await tester.pumpWidget(
      overlay(
        child: const SelectableText(
          kMoreThanFourLines,
          dragStartBehavior: DragStartBehavior.down,
          style: TextStyle(color: Colors.black, fontSize: 34.0),
          maxLines: 2,
        ),
      ),
    );

    final EditableText editableTextWidget = tester.widget(find.byType(EditableText));
    final TextEditingController controller = editableTextWidget.controller;
    RenderBox findInputBox() => tester.renderObject(find.byType(SelectableText));
    final RenderBox inputBox = findInputBox();

    // Check that the last line of text is not displayed.
    final Offset firstPos = textOffsetToPosition(tester, kMoreThanFourLines.indexOf('First'));
    final Offset fourthPos = textOffsetToPosition(tester, kMoreThanFourLines.indexOf('Fourth'));
    expect(firstPos.dx, 0.0);
    expect(fourthPos.dx, 0.0);
    expect(firstPos.dx, fourthPos.dx);
    expect(firstPos.dy, lessThan(fourthPos.dy));
    expect(
      inputBox.hitTest(BoxHitTestResult(), position: inputBox.globalToLocal(firstPos)),
      isTrue,
    );
    expect(
      inputBox.hitTest(BoxHitTestResult(), position: inputBox.globalToLocal(fourthPos)),
      isFalse,
    );

    TestGesture gesture = await tester.startGesture(firstPos, pointer: 7);
    await tester.pump();
    await gesture.moveBy(const Offset(0.0, -1000.0));
    await tester.pump(const Duration(seconds: 1));
    // Wait and drag again to trigger https://github.com/flutter/flutter/issues/6329
    // (No idea why this is necessary, but the bug wouldn't repro without it.)
    await gesture.moveBy(const Offset(0.0, -1000.0));
    await tester.pump(const Duration(seconds: 1));
    await gesture.up();
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    // Now the first line is scrolled up, and the fourth line is visible.
    Offset newFirstPos = textOffsetToPosition(tester, kMoreThanFourLines.indexOf('First'));
    Offset newFourthPos = textOffsetToPosition(tester, kMoreThanFourLines.indexOf('Fourth'));

    expect(newFirstPos.dy, lessThan(firstPos.dy));
    expect(
      inputBox.hitTest(BoxHitTestResult(), position: inputBox.globalToLocal(newFirstPos)),
      isFalse,
    );
    expect(
      inputBox.hitTest(BoxHitTestResult(), position: inputBox.globalToLocal(newFourthPos)),
      isTrue,
    );

    // Now try scrolling by dragging the selection handle.
    // Long press the middle of the word "won't" in the fourth line.
    final Offset selectedWordPos = textOffsetToPosition(
      tester,
      kMoreThanFourLines.indexOf('Fourth line') + 14,
    );

    gesture = await tester.startGesture(selectedWordPos, pointer: 7);
    await tester.pump(const Duration(seconds: 1));
    await gesture.up();
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));

    expect(controller.selection.base.offset, 77);
    expect(controller.selection.extent.offset, 82);
    // Sanity check for the word selected is the intended one.
    expect(
      controller.text.substring(controller.selection.baseOffset, controller.selection.extentOffset),
      "won't",
    );

    final RenderEditable renderEditable = findRenderEditable(tester);
    final List<TextSelectionPoint> endpoints = globalize(
      renderEditable.getEndpointsForSelection(controller.selection),
      renderEditable,
    );
    expect(endpoints.length, 2);

    // Drag the left handle to the first line, just after 'First'.
    final Offset handlePos = endpoints[0].point + const Offset(-1, 1);
    final Offset newHandlePos = textOffsetToPosition(
      tester,
      kMoreThanFourLines.indexOf('First') + 5,
    );
    gesture = await tester.startGesture(handlePos, pointer: 7);
    await tester.pump(const Duration(seconds: 1));
    await gesture.moveTo(newHandlePos + const Offset(0.0, -10.0));
    await tester.pump(const Duration(seconds: 1));
    await gesture.up();
    await tester.pump(const Duration(seconds: 1));

    // The text should have scrolled up with the handle to keep the active
    // cursor visible, back to its original position.
    newFirstPos = textOffsetToPosition(tester, kMoreThanFourLines.indexOf('First'));
    newFourthPos = textOffsetToPosition(tester, kMoreThanFourLines.indexOf('Fourth'));
    expect(newFirstPos.dy, firstPos.dy);
    expect(
      inputBox.hitTest(BoxHitTestResult(), position: inputBox.globalToLocal(newFirstPos)),
      isTrue,
    );
    expect(
      inputBox.hitTest(BoxHitTestResult(), position: inputBox.globalToLocal(newFourthPos)),
      isFalse,
    );
  });

  testWidgets('ScrollBehavior can be overridden', (WidgetTester tester) async {
    await tester.pumpWidget(
      boilerplate(
        child: const SelectableText(
          kMoreThanFourLines,
          dragStartBehavior: DragStartBehavior.down,
          style: TextStyle(color: Colors.black, fontSize: 34.0),
          maxLines: 2,
        ),
      ),
    );
    expect(tester.widget<EditableText>(find.byType(EditableText)).scrollBehavior, isNull);
    expect(tester.widget<Scrollable>(find.byType(Scrollable)).scrollBehavior, isNotNull);

    final ScrollBehavior behavior = const ScrollBehavior()..copyWith(scrollbars: false);
    await tester.pumpWidget(
      boilerplate(
        child: SelectableText(
          kMoreThanFourLines,
          dragStartBehavior: DragStartBehavior.down,
          style: const TextStyle(color: Colors.black, fontSize: 34.0),
          maxLines: 2,
          scrollBehavior: behavior,
        ),
      ),
    );
    expect(tester.widget<EditableText>(find.byType(EditableText)).scrollBehavior, equals(behavior));
    expect(tester.widget<Scrollable>(find.byType(Scrollable)).scrollBehavior, equals(behavior));
  });

  testWidgets('minLines cannot be greater than maxLines', (WidgetTester tester) async {
    expect(
      () async {
        await tester.pumpWidget(
          overlay(
            child: SizedBox(width: 300.0, child: SelectableText('abcd', minLines: 4, maxLines: 3)),
          ),
        );
      },
      throwsA(
        isA<AssertionError>().having(
          (AssertionError error) => error.toString(),
          '.toString()',
          contains("minLines can't be greater than maxLines"),
        ),
      ),
    );
  });

  testWidgets('Selectable height with minLine', (WidgetTester tester) async {
    await tester.pumpWidget(selectableTextBuilder());

    RenderBox findTextBox() => tester.renderObject(find.byType(SelectableText));

    final RenderBox textBox = findTextBox();
    final Size emptyInputSize = textBox.size;

    // Even if the text is a one liner, minimum height of SelectableText will determined by minLines.
    await tester.pumpWidget(
      selectableTextBuilder(text: 'No wrapping here.', minLines: 2, maxLines: 3),
    );
    expect(findTextBox(), equals(textBox));
    expect(textBox.size.height, emptyInputSize.height * 2);
  });

  testWidgets('Can align to center', (WidgetTester tester) async {
    await tester.pumpWidget(
      overlay(
        child: const SizedBox(
          width: 300.0,
          child: SelectableText('abcd', textAlign: TextAlign.center),
        ),
      ),
    );

    final RenderEditable editable = findRenderEditable(tester);

    final Offset topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 2)).topLeft,
    );

    expect(topLeft.dx, equals(399.0));
  });

  testWidgets('Can align to center within center', (WidgetTester tester) async {
    await tester.pumpWidget(
      overlay(
        child: const SizedBox(
          width: 300.0,
          child: Center(child: SelectableText('abcd', textAlign: TextAlign.center)),
        ),
      ),
    );

    final RenderEditable editable = findRenderEditable(tester);

    final Offset topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 2)).topLeft,
    );

    expect(topLeft.dx, equals(399.0));
  });

  testWidgets('Tapping outside SelectableText clears the selection', (WidgetTester tester) async {
    Future<void> setAppLifecycleState(AppLifecycleState state) async {
      final ByteData? message = const StringCodec().encodeMessage(state.toString());
      await TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/lifecycle',
        message,
        (_) {},
      );
    }

    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: Center(
            child: Column(
              children: <Widget>[
                SelectableText('first selectable text'),
                SelectableText('second selectable text'),
              ],
            ),
          ),
        ),
      ),
    );
    // Setting the app lifecycle state to AppLifecycleState.resumed to simulate
    // an applications default running mode, i.e. the application window is focused.
    await setAppLifecycleState(AppLifecycleState.resumed);
    await tester.pumpAndSettle();

    // First tap on the first SelectableText sets the cursor.
    await tester.tapAt(textOffsetToPosition(tester, 5));
    await tester.pumpAndSettle();

    final EditableText editableTextWidgetFirst = tester.widget(find.byType(EditableText).first);
    final TextEditingController controllerA = editableTextWidgetFirst.controller;
    final EditableText editableTextWidgetSecond = tester.widget(find.byType(EditableText).last);
    final TextEditingController controllerB = editableTextWidgetSecond.controller;

    expect(controllerA.selection, const TextSelection.collapsed(offset: 5));
    expect(controllerB.selection, TextRange.empty);

    // Tapping on the second SelectableText sets the cursor on it, and clears the selection from
    // the first SelectableText.
    final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText).last);
    await tester.tapAt(selectableTextStart);
    await tester.pumpAndSettle();
    expect(controllerA.selection, TextRange.empty);
    expect(controllerB.selection, const TextSelection.collapsed(offset: 0));

    // Setting the app lifecycle state to AppLifecycleState.inactive to simulate
    // a lose of window focus. Selection should remain the same.
    await setAppLifecycleState(AppLifecycleState.inactive);
    await tester.pumpAndSettle();
    expect(controllerA.selection, TextRange.empty);
    expect(controllerB.selection, const TextSelection.collapsed(offset: 0));
  });

  testWidgets('Selectable text is skipped during focus traversal', (WidgetTester tester) async {
    final FocusNode firstFieldFocus = FocusNode();
    addTearDown(firstFieldFocus.dispose);
    final FocusNode lastFieldFocus = FocusNode();
    addTearDown(lastFieldFocus.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: Column(
              children: <Widget>[
                TextField(focusNode: firstFieldFocus, autofocus: true),
                const SelectableText('some text'),
                TextField(focusNode: lastFieldFocus),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.pump();

    expect(firstFieldFocus.hasFocus, isTrue);
    expect(lastFieldFocus.hasFocus, isFalse);

    firstFieldFocus.nextFocus();
    await tester.pump();

    // Expecting focus to skip straight to the second field.
    expect(firstFieldFocus.hasFocus, isFalse);
    expect(lastFieldFocus.hasFocus, isTrue);
  });

  testWidgets('Selectable text identifies as text field in semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      const MaterialApp(home: Material(child: Center(child: SelectableText('some text')))),
    );

    expect(
      semantics,
      includesNodeWith(
        flags: <SemanticsFlag>[
          SemanticsFlag.isTextField,
          SemanticsFlag.isReadOnly,
          SemanticsFlag.isMultiline,
        ],
      ),
    );

    semantics.dispose();
  });

  testWidgets('Selectable text rich text with spell out in semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: Center(child: SelectableText.rich(TextSpan(text: 'some text', spellOut: true))),
        ),
      ),
    );

    expect(
      semantics,
      includesNodeWith(
        attributedValue: AttributedString(
          'some text',
          attributes: <StringAttribute>[
            SpellOutStringAttribute(range: const TextRange(start: 0, end: 9)),
          ],
        ),
        flags: <SemanticsFlag>[
          SemanticsFlag.isTextField,
          SemanticsFlag.isReadOnly,
          SemanticsFlag.isMultiline,
        ],
      ),
    );

    semantics.dispose();
  });

  testWidgets('Selectable text rich text with locale in semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: Center(
            child: SelectableText.rich(TextSpan(text: 'some text', locale: Locale('es', 'MX'))),
          ),
        ),
      ),
    );

    expect(
      semantics,
      includesNodeWith(
        attributedValue: AttributedString(
          'some text',
          attributes: <StringAttribute>[
            LocaleStringAttribute(
              range: const TextRange(start: 0, end: 9),
              locale: const Locale('es', 'MX'),
            ),
          ],
        ),
        flags: <SemanticsFlag>[
          SemanticsFlag.isTextField,
          SemanticsFlag.isReadOnly,
          SemanticsFlag.isMultiline,
        ],
      ),
    );

    semantics.dispose();
  });

  testWidgets('Selectable rich text with gesture recognizer has correct semantics', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final TapGestureRecognizer recognizer = TapGestureRecognizer();
    addTearDown(recognizer.dispose);

    await tester.pumpWidget(
      overlay(
        child: SelectableText.rich(
          TextSpan(
            children: <TextSpan>[
              const TextSpan(text: 'text'),
              TextSpan(text: 'link', recognizer: recognizer..onTap = () {}),
            ],
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              actions: <SemanticsAction>[SemanticsAction.longPress],
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  id: 2,
                  children: <TestSemantics>[
                    TestSemantics(id: 3, label: 'text', textDirection: TextDirection.ltr),
                    TestSemantics(
                      id: 4,
                      flags: <SemanticsFlag>[SemanticsFlag.isLink],
                      actions: <SemanticsAction>[SemanticsAction.tap],
                      label: 'link',
                      textDirection: TextDirection.ltr,
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    semantics.dispose();
  });

  group('Keyboard Tests', () {
    late TextEditingController controller;

    Future<void> setupWidget(WidgetTester tester, String text) async {
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: RawKeyboardListener(
              focusNode: focusNode,
              child: SelectableText(text, maxLines: 3),
            ),
          ),
        ),
      );
      await tester.tap(find.byType(SelectableText));
      await tester.pumpAndSettle();
      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      controller = editableTextWidget.controller;
    }

    testWidgets('Shift test 1', (WidgetTester tester) async {
      await setupWidget(tester, 'a big house');

      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.arrowLeft);
      expect(controller.selection.extentOffset - controller.selection.baseOffset, -1);
    }, variant: KeySimulatorTransitModeVariant.all());

    testWidgets('Shift test 2', (WidgetTester tester) async {
      await setupWidget(tester, 'abcdefghi');

      controller.selection = const TextSelection.collapsed(offset: 3);
      await tester.pump();

      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.arrowRight);
      await tester.pumpAndSettle();
      expect(controller.selection.extentOffset - controller.selection.baseOffset, 1);
    }, variant: KeySimulatorTransitModeVariant.all());

    testWidgets('Control Shift test', (WidgetTester tester) async {
      await setupWidget(tester, 'their big house');

      await tester.sendKeyDownEvent(LogicalKeyboardKey.control);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.arrowLeft);

      await tester.pumpAndSettle();

      expect(controller.selection.extentOffset - controller.selection.baseOffset, -5);
    }, variant: KeySimulatorTransitModeVariant.all());

    testWidgets('Down and up test', (WidgetTester tester) async {
      await setupWidget(tester, 'a big house');

      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.arrowUp);
      await tester.pumpAndSettle();

      expect(controller.selection.extentOffset - controller.selection.baseOffset, -11);

      await tester.sendKeyUpEvent(LogicalKeyboardKey.arrowUp);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      await tester.sendKeyDownEvent(LogicalKeyboardKey.arrowDown);
      await tester.pumpAndSettle();

      expect(controller.selection.extentOffset - controller.selection.baseOffset, 0);
    }, variant: KeySimulatorTransitModeVariant.all());

    testWidgets('Down and up test 2', (WidgetTester tester) async {
      await setupWidget(tester, 'a big house\njumped over a mouse\nOne more line yay');

      controller.selection = const TextSelection.collapsed(offset: 0);
      await tester.pump();

      for (int i = 0; i < 5; i += 1) {
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
        await tester.pumpAndSettle();
      }
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.pumpAndSettle();
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
      await tester.pumpAndSettle();

      expect(controller.selection.extentOffset - controller.selection.baseOffset, 12);

      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.pumpAndSettle();
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
      await tester.pumpAndSettle();

      expect(controller.selection.extentOffset - controller.selection.baseOffset, 32);

      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      await tester.pumpAndSettle();
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
      await tester.pumpAndSettle();

      expect(controller.selection.extentOffset - controller.selection.baseOffset, 12);

      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      await tester.pumpAndSettle();
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
      await tester.pumpAndSettle();

      expect(controller.selection.extentOffset - controller.selection.baseOffset, 0);

      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      await tester.pumpAndSettle();
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
      await tester.pumpAndSettle();

      expect(controller.selection.extentOffset - controller.selection.baseOffset, -5);
    }, variant: KeySimulatorTransitModeVariant.all());
  });

  testWidgets('Copy test', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    String clipboardContent = '';
    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.platform, (
      MethodCall methodCall,
    ) async {
      if (methodCall.method == 'Clipboard.setData') {
        clipboardContent = (methodCall.arguments as Map<String, dynamic>)['text'] as String;
      } else if (methodCall.method == 'Clipboard.getData') {
        return <String, dynamic>{'text': clipboardContent};
      }
      return null;
    });
    const String testValue = 'a big house\njumped over a mouse';
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: RawKeyboardListener(
            focusNode: focusNode,
            child: const SelectableText(testValue, maxLines: 3),
          ),
        ),
      ),
    );
    final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
    final TextEditingController controller = editableTextWidget.controller;
    focusNode.requestFocus();
    await tester.pump();

    await tester.tap(find.byType(SelectableText));
    await tester.pumpAndSettle();

    controller.selection = const TextSelection.collapsed(offset: 0);
    await tester.pump();

    // Select the first 5 characters.
    await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
    for (int i = 0; i < 5; i += 1) {
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pumpAndSettle();
    }
    await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);

    // Copy them.
    await tester.sendKeyDownEvent(LogicalKeyboardKey.controlRight);
    await tester.sendKeyDownEvent(LogicalKeyboardKey.keyC);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.keyC);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.controlRight);
    await tester.pumpAndSettle();

    expect(clipboardContent, 'a big');

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pumpAndSettle();
  }, variant: KeySimulatorTransitModeVariant.all());

  testWidgets('Select all test', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);
    const String testValue = 'a big house\njumped over a mouse';
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: RawKeyboardListener(
            focusNode: focusNode,
            child: const SelectableText(testValue, maxLines: 3),
          ),
        ),
      ),
    );
    final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
    final TextEditingController controller = editableTextWidget.controller;
    focusNode.requestFocus();
    await tester.pump();

    await tester.tap(find.byType(SelectableText));
    await tester.pumpAndSettle();

    // Select All.
    await tester.sendKeyDownEvent(LogicalKeyboardKey.control);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyA);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.control);
    await tester.pumpAndSettle();

    expect(controller.selection.baseOffset, 0);
    expect(controller.selection.extentOffset, 31);
  }, variant: KeySimulatorTransitModeVariant.all());

  testWidgets('keyboard selection should call onSelectionChanged', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);
    TextSelection? newSelection;
    const String testValue = 'a big house\njumped over a mouse';
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: RawKeyboardListener(
            focusNode: focusNode,
            child: SelectableText(
              testValue,
              maxLines: 3,
              onSelectionChanged: (TextSelection selection, SelectionChangedCause? cause) {
                expect(newSelection, isNull);
                newSelection = selection;
              },
            ),
          ),
        ),
      ),
    );
    final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
    final TextEditingController controller = editableTextWidget.controller;
    focusNode.requestFocus();
    await tester.pump();

    await tester.tap(find.byType(SelectableText));
    await tester.pumpAndSettle();
    expect(newSelection!.baseOffset, 31);
    expect(newSelection!.extentOffset, 31);
    newSelection = null;

    controller.selection = const TextSelection.collapsed(offset: 0);
    await tester.pump();

    // Select the first 5 characters.
    await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
    for (int i = 0; i < 5; i += 1) {
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pumpAndSettle();
      expect(newSelection!.baseOffset, 0);
      expect(newSelection!.extentOffset, i + 1);
      newSelection = null;
    }
  }, variant: KeySimulatorTransitModeVariant.all());

  testWidgets('Changing positions of selectable text', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);
    final List<KeyEvent> events = <KeyEvent>[];

    final Key key1 = UniqueKey();
    final Key key2 = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: KeyboardListener(
            focusNode: focusNode,
            onKeyEvent: events.add,
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: <Widget>[
                SelectableText('a big house', key: key1, maxLines: 3),
                SelectableText('another big house', key: key2, maxLines: 3),
              ],
            ),
          ),
        ),
      ),
    );

    EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
    TextEditingController c1 = editableTextWidget.controller;

    await tester.tap(find.byType(EditableText).first);
    await tester.pumpAndSettle();

    await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
    for (int i = 0; i < 5; i += 1) {
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pumpAndSettle();
    }
    await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
    await tester.pumpAndSettle();

    expect(c1.selection.extentOffset - c1.selection.baseOffset, -5);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: KeyboardListener(
            focusNode: focusNode,
            onKeyEvent: events.add,
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: <Widget>[
                SelectableText('another big house', key: key2, maxLines: 3),
                SelectableText('a big house', key: key1, maxLines: 3),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
    for (int i = 0; i < 5; i += 1) {
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    }
    await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
    await tester.pumpAndSettle();

    editableTextWidget = tester.widget(find.byType(EditableText).last);
    c1 = editableTextWidget.controller;

    expect(c1.selection.extentOffset - c1.selection.baseOffset, -10);
  }, variant: KeySimulatorTransitModeVariant.all());

  testWidgets('Changing focus test', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);
    final List<KeyEvent> events = <KeyEvent>[];

    final Key key1 = UniqueKey();
    final Key key2 = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: KeyboardListener(
            focusNode: focusNode,
            onKeyEvent: events.add,
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: <Widget>[
                SelectableText('a big house', key: key1, maxLines: 3),
                SelectableText('another big house', key: key2, maxLines: 3),
              ],
            ),
          ),
        ),
      ),
    );

    final EditableText editableTextWidget1 = tester.widget(find.byType(EditableText).first);
    final TextEditingController c1 = editableTextWidget1.controller;

    final EditableText editableTextWidget2 = tester.widget(find.byType(EditableText).last);
    final TextEditingController c2 = editableTextWidget2.controller;

    await tester.tap(find.byType(SelectableText).first);
    await tester.pumpAndSettle();

    await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
    for (int i = 0; i < 5; i += 1) {
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pumpAndSettle();
    }
    await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
    await tester.pumpAndSettle();

    expect(c1.selection.extentOffset - c1.selection.baseOffset, -5);
    expect(c2.selection.extentOffset - c2.selection.baseOffset, 0);

    await tester.tap(find.byType(SelectableText).last);
    await tester.pumpAndSettle();

    await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
    for (int i = 0; i < 5; i += 1) {
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pumpAndSettle();
    }
    await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
    await tester.pumpAndSettle();

    expect(c1.selection.extentOffset - c1.selection.baseOffset, -5);
    expect(c2.selection.extentOffset - c2.selection.baseOffset, -5);
  }, variant: KeySimulatorTransitModeVariant.all());

  testWidgets('Caret works when maxLines is null', (WidgetTester tester) async {
    await tester.pumpWidget(overlay(child: const SelectableText('x')));

    final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
    final TextEditingController controller = editableTextWidget.controller;

    expect(controller.selection.baseOffset, -1);

    // Tap the selection handle to bring up the "paste / select all" menu.
    await tester.tapAt(textOffsetToPosition(tester, 0));
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 200),
    ); // skip past the frame where the opacity is

    // Confirm that the selection was updated.
    expect(controller.selection.baseOffset, 0);
  });

  testWidgets('SelectableText baseline alignment no-strut', (WidgetTester tester) async {
    final Key keyA = UniqueKey();
    final Key keyB = UniqueKey();

    await tester.pumpWidget(
      overlay(
        child: Row(
          crossAxisAlignment: CrossAxisAlignment.baseline,
          textBaseline: TextBaseline.alphabetic,
          children: <Widget>[
            Expanded(
              child: SelectableText(
                'A',
                key: keyA,
                style: const TextStyle(fontFamily: 'FlutterTest', fontSize: 10.0),
                strutStyle: StrutStyle.disabled,
              ),
            ),
            const Text('abc', style: TextStyle(fontFamily: 'FlutterTest', fontSize: 20.0)),
            Expanded(
              child: SelectableText(
                'B',
                key: keyB,
                style: const TextStyle(fontFamily: 'FlutterTest', fontSize: 30.0),
                strutStyle: StrutStyle.disabled,
              ),
            ),
          ],
        ),
      ),
    );

    // The test font extends 0.25 * fontSize below the baseline.
    // So the three row elements line up like this:
    //
    //  A    abc  B
    //  -------------  baseline
    //  2.5  5    7.5  space below the baseline = 0.25 * fontSize
    //  -------------  rowBottomY

    final double rowBottomY = tester.getBottomLeft(find.byType(Row)).dy;
    expect(tester.getBottomLeft(find.byKey(keyA)).dy, rowBottomY - 5.0);
    expect(tester.getBottomLeft(find.text('abc')).dy, rowBottomY - 2.5);
    expect(tester.getBottomLeft(find.byKey(keyB)).dy, rowBottomY);
  });

  testWidgets('SelectableText baseline alignment', (WidgetTester tester) async {
    final Key keyA = UniqueKey();
    final Key keyB = UniqueKey();

    await tester.pumpWidget(
      overlay(
        child: Row(
          crossAxisAlignment: CrossAxisAlignment.baseline,
          textBaseline: TextBaseline.alphabetic,
          children: <Widget>[
            Expanded(
              child: SelectableText(
                'A',
                key: keyA,
                style: const TextStyle(fontFamily: 'FlutterTest', fontSize: 10.0),
              ),
            ),
            const Text('abc', style: TextStyle(fontFamily: 'FlutterTest', fontSize: 20.0)),
            Expanded(
              child: SelectableText(
                'B',
                key: keyB,
                style: const TextStyle(fontFamily: 'FlutterTest', fontSize: 30.0),
              ),
            ),
          ],
        ),
      ),
    );

    // The test font extends 0.25 * fontSize below the baseline.
    // So the three row elements line up like this:
    //
    //  A    abc  B
    //  -------------  baseline
    //  2.5  5    7.5  space below the baseline = 0.25 * fontSize
    //  -------------  rowBottomY

    final double rowBottomY = tester.getBottomLeft(find.byType(Row)).dy;
    expect(tester.getBottomLeft(find.byKey(keyA)).dy, rowBottomY - 5.0);
    expect(tester.getBottomLeft(find.text('abc')).dy, rowBottomY - 2.5);
    expect(tester.getBottomLeft(find.byKey(keyB)).dy, rowBottomY);
  });

  testWidgets('SelectableText semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final Key key = UniqueKey();

    await tester.pumpWidget(overlay(child: SelectableText('Guten Tag', key: key)));

    final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
    final TextEditingController controller = editableTextWidget.controller;

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              textDirection: TextDirection.ltr,
              value: 'Guten Tag',
              actions: <SemanticsAction>[SemanticsAction.longPress],
              flags: <SemanticsFlag>[
                SemanticsFlag.isTextField,
                SemanticsFlag.isReadOnly,
                SemanticsFlag.isMultiline,
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    await tester.tap(find.byKey(key));
    await tester.pump();

    controller.selection = const TextSelection.collapsed(offset: 9);
    await tester.pump();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              textDirection: TextDirection.ltr,
              value: 'Guten Tag',
              textSelection: const TextSelection.collapsed(offset: 9),
              actions: <SemanticsAction>[
                SemanticsAction.longPress,
                SemanticsAction.moveCursorBackwardByCharacter,
                SemanticsAction.moveCursorBackwardByWord,
                SemanticsAction.setSelection,
              ],
              flags: <SemanticsFlag>[
                SemanticsFlag.isReadOnly,
                SemanticsFlag.isTextField,
                SemanticsFlag.isMultiline,
                SemanticsFlag.isFocused,
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    controller.selection = const TextSelection.collapsed(offset: 4);
    await tester.pump();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              textDirection: TextDirection.ltr,
              textSelection: const TextSelection.collapsed(offset: 4),
              value: 'Guten Tag',
              actions: <SemanticsAction>[
                SemanticsAction.longPress,
                SemanticsAction.moveCursorBackwardByCharacter,
                SemanticsAction.moveCursorForwardByCharacter,
                SemanticsAction.moveCursorBackwardByWord,
                SemanticsAction.moveCursorForwardByWord,
                SemanticsAction.setSelection,
              ],
              flags: <SemanticsFlag>[
                SemanticsFlag.isReadOnly,
                SemanticsFlag.isTextField,
                SemanticsFlag.isMultiline,
                SemanticsFlag.isFocused,
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    controller.selection = const TextSelection.collapsed(offset: 0);
    await tester.pump();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              textDirection: TextDirection.ltr,
              textSelection: const TextSelection.collapsed(offset: 0),
              value: 'Guten Tag',
              actions: <SemanticsAction>[
                SemanticsAction.longPress,
                SemanticsAction.moveCursorForwardByCharacter,
                SemanticsAction.moveCursorForwardByWord,
                SemanticsAction.setSelection,
              ],
              flags: <SemanticsFlag>[
                SemanticsFlag.isReadOnly,
                SemanticsFlag.isTextField,
                SemanticsFlag.isMultiline,
                SemanticsFlag.isFocused,
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('SelectableText semantics, with semanticsLabel', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final Key key = UniqueKey();

    await tester.pumpWidget(
      overlay(
        child: SelectableText(
          'Guten Tag',
          semanticsLabel: 'German greeting for good day',
          key: key,
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: 1,
              actions: <SemanticsAction>[SemanticsAction.longPress],
              label: 'German greeting for good day',
              textDirection: TextDirection.ltr,
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );
    semantics.dispose();
  });

  testWidgets('SelectableText semantics, enableInteractiveSelection = false', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final Key key = UniqueKey();

    await tester.pumpWidget(
      overlay(child: SelectableText('Guten Tag', key: key, enableInteractiveSelection: false)),
    );

    await tester.tap(find.byKey(key));
    await tester.pump();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              value: 'Guten Tag',
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[
                SemanticsAction.longPress,
                // Absent the following because enableInteractiveSelection: false
                // SemanticsAction.moveCursorBackwardByCharacter,
                // SemanticsAction.moveCursorBackwardByWord,
                // SemanticsAction.setSelection,
                // SemanticsAction.paste,
              ],
              flags: <SemanticsFlag>[
                SemanticsFlag.isReadOnly,
                SemanticsFlag.isTextField,
                SemanticsFlag.isMultiline,
                // SelectableText act like a text widget when enableInteractiveSelection
                // is false. It will not respond to any pointer event.
                // SemanticsFlag.isFocused,
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('SelectableText semantics for selections', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final Key key = UniqueKey();

    await tester.pumpWidget(overlay(child: SelectableText('Hello', key: key)));

    final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
    final TextEditingController controller = editableTextWidget.controller;

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              value: 'Hello',
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[SemanticsAction.longPress],
              flags: <SemanticsFlag>[
                SemanticsFlag.isReadOnly,
                SemanticsFlag.isTextField,
                SemanticsFlag.isMultiline,
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    // Focus the selectable text
    await tester.tap(find.byKey(key));
    await tester.pump();

    controller.selection = const TextSelection.collapsed(offset: 5);
    await tester.pump();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              value: 'Hello',
              textSelection: const TextSelection.collapsed(offset: 5),
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[
                SemanticsAction.longPress,
                SemanticsAction.moveCursorBackwardByCharacter,
                SemanticsAction.moveCursorBackwardByWord,
                SemanticsAction.setSelection,
              ],
              flags: <SemanticsFlag>[
                SemanticsFlag.isReadOnly,
                SemanticsFlag.isTextField,
                SemanticsFlag.isMultiline,
                SemanticsFlag.isFocused,
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    controller.selection = const TextSelection(baseOffset: 5, extentOffset: 3);
    await tester.pump();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              value: 'Hello',
              textSelection: const TextSelection(baseOffset: 5, extentOffset: 3),
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[
                SemanticsAction.longPress,
                SemanticsAction.moveCursorBackwardByCharacter,
                SemanticsAction.moveCursorForwardByCharacter,
                SemanticsAction.moveCursorBackwardByWord,
                SemanticsAction.moveCursorForwardByWord,
                SemanticsAction.setSelection,
                SemanticsAction.copy,
              ],
              flags: <SemanticsFlag>[
                SemanticsFlag.isReadOnly,
                SemanticsFlag.isTextField,
                SemanticsFlag.isMultiline,
                SemanticsFlag.isFocused,
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('semantic nodes of offscreen recognizers are marked hidden', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/100395.
    final SemanticsTester semantics = SemanticsTester(tester);
    const TextStyle textStyle = TextStyle(fontSize: 200);
    const String onScreenText = 'onscreen\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n';
    const String offScreenText = 'off screen';
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);
    final TapGestureRecognizer recognizer = TapGestureRecognizer();
    addTearDown(recognizer.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: SingleChildScrollView(
          controller: controller,
          child: SelectableText.rich(
            TextSpan(
              children: <TextSpan>[
                const TextSpan(text: onScreenText),
                TextSpan(text: offScreenText, recognizer: recognizer..onTap = () {}),
              ],
              style: textStyle,
            ),
            textDirection: TextDirection.ltr,
          ),
        ),
      ),
    );

    final TestSemantics expectedSemantics = TestSemantics.root(
      children: <TestSemantics>[
        TestSemantics(
          textDirection: TextDirection.ltr,
          children: <TestSemantics>[
            TestSemantics(
              children: <TestSemantics>[
                TestSemantics(
                  flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                  children: <TestSemantics>[
                    TestSemantics(
                      flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                      actions: <SemanticsAction>[
                        SemanticsAction.scrollUp,
                        SemanticsAction.scrollToOffset,
                      ],
                      children: <TestSemantics>[
                        TestSemantics(
                          actions: <SemanticsAction>[SemanticsAction.longPress],
                          children: <TestSemantics>[
                            TestSemantics(
                              children: <TestSemantics>[
                                TestSemantics(
                                  label: 'onscreen\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n',
                                  textDirection: TextDirection.ltr,
                                ),
                                TestSemantics(
                                  flags: <SemanticsFlag>[
                                    SemanticsFlag.isHidden,
                                    SemanticsFlag.isLink,
                                  ],
                                  actions: <SemanticsAction>[SemanticsAction.tap],
                                  label: 'off screen',
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
        ),
      ],
    );
    expect(
      semantics,
      hasSemantics(expectedSemantics, ignoreTransform: true, ignoreId: true, ignoreRect: true),
    );

    // Test shows on screen.
    expect(controller.offset, 0.0);
    tester.binding.pipelineOwner.semanticsOwner!.performAction(8, SemanticsAction.showOnScreen);
    await tester.pumpAndSettle();
    expect(controller.offset != 0.0, isTrue);

    semantics.dispose();
  });

  testWidgets('SelectableText change selection with semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final SemanticsOwner semanticsOwner = tester.binding.pipelineOwner.semanticsOwner!;
    final Key key = UniqueKey();

    await tester.pumpWidget(overlay(child: SelectableText('Hello', key: key)));

    final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
    final TextEditingController controller = editableTextWidget.controller;

    // Focus the selectable text.
    await tester.tap(find.byKey(key));
    await tester.pump();

    controller.selection = const TextSelection(baseOffset: 5, extentOffset: 5);
    await tester.pump();

    const int inputFieldId = 1;

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: inputFieldId,
              value: 'Hello',
              textSelection: const TextSelection.collapsed(offset: 5),
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[
                SemanticsAction.longPress,
                SemanticsAction.moveCursorBackwardByCharacter,
                SemanticsAction.moveCursorBackwardByWord,
                SemanticsAction.setSelection,
              ],
              flags: <SemanticsFlag>[
                SemanticsFlag.isReadOnly,
                SemanticsFlag.isTextField,
                SemanticsFlag.isMultiline,
                SemanticsFlag.isFocused,
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    // Move cursor back once.
    semanticsOwner.performAction(inputFieldId, SemanticsAction.setSelection, <dynamic, dynamic>{
      'base': 4,
      'extent': 4,
    });
    await tester.pump();
    expect(controller.selection, const TextSelection.collapsed(offset: 4));

    // Move cursor to front.
    semanticsOwner.performAction(inputFieldId, SemanticsAction.setSelection, <dynamic, dynamic>{
      'base': 0,
      'extent': 0,
    });
    await tester.pump();
    expect(controller.selection, const TextSelection.collapsed(offset: 0));

    // Select all.
    semanticsOwner.performAction(inputFieldId, SemanticsAction.setSelection, <dynamic, dynamic>{
      'base': 0,
      'extent': 5,
    });
    await tester.pump();
    expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 5));
    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: inputFieldId,
              value: 'Hello',
              textSelection: const TextSelection(baseOffset: 0, extentOffset: 5),
              textDirection: TextDirection.ltr,
              actions: <SemanticsAction>[
                SemanticsAction.longPress,
                SemanticsAction.moveCursorBackwardByCharacter,
                SemanticsAction.moveCursorBackwardByWord,
                SemanticsAction.setSelection,
                SemanticsAction.copy,
              ],
              flags: <SemanticsFlag>[
                SemanticsFlag.isReadOnly,
                SemanticsFlag.isTextField,
                SemanticsFlag.isMultiline,
                SemanticsFlag.isFocused,
              ],
            ),
          ],
        ),
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('Can activate SelectableText with explicit controller via semantics', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/17801

    const String testValue = 'Hello';

    final SemanticsTester semantics = SemanticsTester(tester);
    final SemanticsOwner semanticsOwner = tester.binding.pipelineOwner.semanticsOwner!;
    final Key key = UniqueKey();

    await tester.pumpWidget(overlay(child: SelectableText(testValue, key: key)));

    const int inputFieldId = 1;

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: inputFieldId,
              flags: <SemanticsFlag>[
                SemanticsFlag.isReadOnly,
                SemanticsFlag.isTextField,
                SemanticsFlag.isMultiline,
              ],
              actions: <SemanticsAction>[SemanticsAction.longPress],
              value: testValue,
              textDirection: TextDirection.ltr,
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    semanticsOwner.performAction(inputFieldId, SemanticsAction.longPress);
    await tester.pump();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: inputFieldId,
              flags: <SemanticsFlag>[
                SemanticsFlag.isReadOnly,
                SemanticsFlag.isTextField,
                SemanticsFlag.isMultiline,
                SemanticsFlag.isFocused,
              ],
              actions: <SemanticsAction>[
                SemanticsAction.longPress,
                SemanticsAction.moveCursorBackwardByCharacter,
                SemanticsAction.moveCursorBackwardByWord,
                SemanticsAction.setSelection,
              ],
              value: testValue,
              textDirection: TextDirection.ltr,
              textSelection: const TextSelection(
                baseOffset: testValue.length,
                extentOffset: testValue.length,
              ),
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('onTap is called upon tap', (WidgetTester tester) async {
    int tapCount = 0;
    await tester.pumpWidget(
      overlay(
        child: SelectableText(
          'something',
          onTap: () {
            tapCount += 1;
          },
        ),
      ),
    );

    expect(tapCount, 0);
    await tester.tap(find.byType(SelectableText));
    // Wait a bit so they're all single taps and not double taps.
    await tester.pump(const Duration(milliseconds: 300));
    await tester.tap(find.byType(SelectableText));
    await tester.pump(const Duration(milliseconds: 300));
    await tester.tap(find.byType(SelectableText));
    await tester.pump(const Duration(milliseconds: 300));
    expect(tapCount, 3);
  });

  testWidgets('SelectableText style is merged with default text style', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/23994
    final TextStyle defaultStyle = TextStyle(color: Colors.blue[500]);
    Widget buildFrame(TextStyle style) {
      return MaterialApp(
        home: Material(
          child: DefaultTextStyle(
            style: defaultStyle,
            child: Center(child: SelectableText('something', style: style)),
          ),
        ),
      );
    }

    // Empty TextStyle is overridden by theme.
    await tester.pumpWidget(buildFrame(const TextStyle()));
    EditableText editableText = tester.widget(find.byType(EditableText));
    expect(editableText.style.color, defaultStyle.color);
    expect(editableText.style.background, defaultStyle.background);
    expect(editableText.style.shadows, defaultStyle.shadows);
    expect(editableText.style.decoration, defaultStyle.decoration);
    expect(editableText.style.locale, defaultStyle.locale);
    expect(editableText.style.wordSpacing, defaultStyle.wordSpacing);

    // Properties set on TextStyle override theme.
    const Color setColor = Colors.red;
    await tester.pumpWidget(buildFrame(const TextStyle(color: setColor)));
    editableText = tester.widget(find.byType(EditableText));
    expect(editableText.style.color, setColor);

    // inherit: false causes nothing to be merged in from theme.
    await tester.pumpWidget(
      buildFrame(
        const TextStyle(fontSize: 24.0, textBaseline: TextBaseline.alphabetic, inherit: false),
      ),
    );
    editableText = tester.widget(find.byType(EditableText));
    expect(editableText.style.color, isNull);
  });

  testWidgets('style enforces required fields', (WidgetTester tester) async {
    Widget buildFrame(TextStyle style) {
      return MaterialApp(home: Material(child: SelectableText('something', style: style)));
    }

    await tester.pumpWidget(
      buildFrame(
        const TextStyle(inherit: false, fontSize: 12.0, textBaseline: TextBaseline.alphabetic),
      ),
    );
    expect(tester.takeException(), isNull);

    // With inherit not set to false, will pickup required fields from theme.
    await tester.pumpWidget(buildFrame(const TextStyle(fontSize: 12.0)));
    expect(tester.takeException(), isNull);

    await tester.pumpWidget(buildFrame(const TextStyle(inherit: false, fontSize: 12.0)));
    expect(tester.takeException(), isNotNull);
  });

  testWidgets(
    'tap moves cursor to the edge of the word it tapped',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Center(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      await tester.tapAt(selectableTextStart + const Offset(50.0, 5.0));
      await tester.pump();

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;
      // We moved the cursor.
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 7, affinity: TextAffinity.upstream),
      );

      // But don't trigger the toolbar.
      expect(find.byType(CupertinoButton), findsNothing);
    },
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );

  testWidgets(
    'tap moves cursor to the position tapped (Android)',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Center(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      await tester.tapAt(selectableTextStart + const Offset(50.0, 5.0));
      await tester.pump();

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      // We moved the cursor.
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 4, affinity: TextAffinity.upstream),
      );

      // But don't trigger the toolbar.
      expect(find.byType(TextButton), findsNothing);
    },
    variant: TargetPlatformVariant.all(excluding: const <TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'two slow taps do not trigger a word selection on iOS',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Center(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      await tester.tapAt(selectableTextStart + const Offset(50.0, 5.0));
      await tester.pump(const Duration(milliseconds: 500));
      await tester.tapAt(selectableTextStart + const Offset(50.0, 5.0));
      await tester.pump();

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      // Plain collapsed selection.
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 7, affinity: TextAffinity.upstream),
      );

      // No toolbar.
      expect(find.byType(CupertinoButton), findsNothing);
    },
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );

  testWidgets(
    'two slow taps do not trigger a word selection',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Center(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      await tester.tapAt(selectableTextStart + const Offset(50.0, 5.0));
      await tester.pump(const Duration(milliseconds: 500));
      await tester.tapAt(selectableTextStart + const Offset(50.0, 5.0));
      await tester.pump();

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      // Plain collapsed selection.
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 4, affinity: TextAffinity.upstream),
      );

      // No toolbar.
      expect(find.byType(CupertinoButton), findsNothing);
    },
    variant: TargetPlatformVariant.all(excluding: <TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'double tap selects word and first tap of double tap moves cursor',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Center(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      // This tap just puts the cursor somewhere different than where the double
      // tap will occur to test that the double tap moves the existing cursor first.
      await tester.tapAt(selectableTextStart + const Offset(50.0, 5.0));
      await tester.pump(const Duration(milliseconds: 500));

      await tester.tapAt(selectableTextStart + const Offset(150.0, 5.0));
      await tester.pump(const Duration(milliseconds: 50));

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      // First tap moved the cursor.
      // On iOS, this moves the cursor to the closest word edge.
      // On macOS, this moves the cursor to the tapped position.
      expect(
        controller.selection,
        TextSelection.collapsed(
          offset: defaultTargetPlatform == TargetPlatform.iOS ? 12 : 11,
          affinity: TextAffinity.upstream,
        ),
      );
      await tester.tapAt(selectableTextStart + const Offset(150.0, 5.0));
      await tester.pump();

      // Second tap selects the word around the cursor.
      expect(controller.selection, const TextSelection(baseOffset: 8, extentOffset: 12));

      expectCupertinoSelectionToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'double tap selects word and first tap of double tap moves cursor and shows toolbar (Android)',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Center(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      // This tap just puts the cursor somewhere different than where the double
      // tap will occur to test that the double tap moves the existing cursor first.
      await tester.tapAt(selectableTextStart + const Offset(50.0, 5.0));
      await tester.pump(const Duration(milliseconds: 500));

      await tester.tapAt(selectableTextStart + const Offset(150.0, 5.0));
      await tester.pump(const Duration(milliseconds: 50));

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      // First tap moved the cursor.
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 11, affinity: TextAffinity.upstream),
      );
      await tester.tapAt(selectableTextStart + const Offset(150.0, 5.0));
      await tester.pump();

      // Second tap selects the word around the cursor.
      expect(controller.selection, const TextSelection(baseOffset: 8, extentOffset: 12));

      expectMaterialSelectionToolbar();
    },
  );

  testWidgets('double tap on top of cursor also selects word (Android)', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(child: Center(child: SelectableText('Atwater Peel Sherbrooke Bonaventure'))),
      ),
    );

    // Tap to put the cursor after the "w".
    const int index = 3;
    await tester.tapAt(textOffsetToPosition(tester, index));
    await tester.pump(const Duration(milliseconds: 500));

    final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
    final TextEditingController controller = editableTextWidget.controller;

    expect(controller.selection, const TextSelection.collapsed(offset: index));

    // Double tap on the same location.
    await tester.tapAt(textOffsetToPosition(tester, index));
    await tester.pump(const Duration(milliseconds: 50));

    // First tap doesn't change the selection.
    expect(controller.selection, const TextSelection.collapsed(offset: index));

    // Second tap selects the word around the cursor.
    await tester.tapAt(textOffsetToPosition(tester, index));
    await tester.pump();
    expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));

    expectMaterialSelectionToolbar();
  });

  testWidgets(
    'double tap hold selects word',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Center(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      await tester.tapAt(selectableTextStart + const Offset(150.0, 5.0));
      await tester.pump(const Duration(milliseconds: 50));
      final TestGesture gesture = await tester.startGesture(
        selectableTextStart + const Offset(150.0, 5.0),
      );
      // Hold the press.
      await tester.pump(const Duration(milliseconds: 500));

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      expect(controller.selection, const TextSelection(baseOffset: 8, extentOffset: 12));

      expectCupertinoSelectionToolbar();

      await gesture.up();
      await tester.pump();

      // Still selected.
      expect(controller.selection, const TextSelection(baseOffset: 8, extentOffset: 12));
      // The toolbar is still showing.
      expectCupertinoSelectionToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'double tap selects word with semantics label',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Center(
              child: SelectableText.rich(
                TextSpan(text: 'Atwater Peel Sherbrooke Bonaventure', semanticsLabel: ''),
              ),
            ),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      await tester.tapAt(selectableTextStart + const Offset(220.0, 5.0));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(selectableTextStart + const Offset(220.0, 5.0));
      await tester.pump();

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      expect(controller.selection, const TextSelection(baseOffset: 13, extentOffset: 23));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'tap after a double tap select is not affected (iOS)',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Center(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      await tester.tapAt(selectableTextStart + const Offset(150.0, 5.0));
      await tester.pump(const Duration(milliseconds: 50));

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      // First tap moved the cursor.
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 12, affinity: TextAffinity.upstream),
      );
      await tester.tapAt(selectableTextStart + const Offset(150.0, 5.0));
      await tester.pump(const Duration(milliseconds: 500));

      await tester.tapAt(selectableTextStart + const Offset(100.0, 5.0));
      await tester.pump();

      // Plain collapsed selection at the edge of first word. In iOS 12, the
      // first tap after a double tap ends up putting the cursor at where
      // you tapped instead of the edge like every other single tap. This is
      // likely a bug in iOS 12 and not present in other versions.
      expect(controller.selection, const TextSelection.collapsed(offset: 7));

      // No toolbar.
      expect(find.byType(CupertinoButton), findsNothing);
    },
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );

  testWidgets(
    'tap after a double tap select is not affected (macOS)',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Center(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      await tester.tapAt(selectableTextStart + const Offset(150.0, 5.0));
      await tester.pump(const Duration(milliseconds: 50));

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      // First tap moved the cursor.
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 11, affinity: TextAffinity.upstream),
      );
      await tester.tapAt(selectableTextStart + const Offset(150.0, 5.0));
      await tester.pump(const Duration(milliseconds: 500));

      await tester.tapAt(selectableTextStart + const Offset(100.0, 5.0));
      await tester.pump();

      // Collapse selection.
      expect(controller.selection, const TextSelection.collapsed(offset: 7));

      // No toolbar.
      expect(find.byType(CupertinoButton), findsNothing);
    },
    variant: TargetPlatformVariant.only(TargetPlatform.macOS),
  );

  testWidgets(
    'long press selects word and shows toolbar (iOS)',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Center(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      await tester.longPressAt(selectableTextStart + const Offset(50.0, 5.0));
      await tester.pump();

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      // The long pressed word is selected.
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));

      expectCupertinoSelectionToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('long press selects word and shows toolbar (Android)', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(child: Center(child: SelectableText('Atwater Peel Sherbrooke Bonaventure'))),
      ),
    );

    final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

    await tester.longPressAt(selectableTextStart + const Offset(50.0, 5.0));
    await tester.pump();

    final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
    final TextEditingController controller = editableTextWidget.controller;

    expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));

    expectMaterialSelectionToolbar();
  });

  testWidgets(
    'long press selects word and shows custom toolbar (Cupertino)',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: SelectableText(
                'Atwater Peel Sherbrooke Bonaventure',
                selectionControls: cupertinoTextSelectionControls,
              ),
            ),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      await tester.longPressAt(selectableTextStart + const Offset(50.0, 5.0));
      await tester.pump();

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      // The long pressed word is selected.
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));

      // Toolbar shows one button (copy).
      expect(find.byType(CupertinoButton), findsNWidgets(1));
      expect(find.text('Copy'), findsOneWidget);
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'long press selects word and shows custom toolbar (Material)',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: SelectableText(
                'Atwater Peel Sherbrooke Bonaventure',
                selectionControls: materialTextSelectionControls,
              ),
            ),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      await tester.longPressAt(selectableTextStart + const Offset(50.0, 5.0));
      await tester.pump();

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));

      // Collapsed toolbar shows 2 buttons: copy, select all
      expect(find.byType(TextButton), findsNWidgets(2));
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Select all'), findsOneWidget);
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets('textSelectionControls is passed to EditableText', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Scaffold(
            body: SelectableText(
              'Atwater Peel Sherbrooke Bonaventure',
              selectionControls: materialTextSelectionControls,
            ),
          ),
        ),
      ),
    );

    final EditableText widget = tester.widget(find.byType(EditableText));
    expect(widget.selectionControls, equals(materialTextSelectionControls));
  });

  testWidgets(
    'PageView beats SelectableText drag gestures (iOS)',
    (WidgetTester tester) async {
      // This is a regression test for
      // https://github.com/flutter/flutter/issues/130198.
      final PageController pageController = PageController();
      addTearDown(pageController.dispose);
      const String testValue = 'abc def ghi jkl mno pqr stu vwx yz';

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: PageView(
              controller: pageController,
              children: const <Widget>[
                Center(child: SelectableText(testValue)),
                SizedBox(height: 200.0, child: Center(child: Text('Page 2'))),
              ],
            ),
          ),
        ),
      );

      await skipPastScrollingAnimation(tester);

      final Offset gPos = textOffsetToPosition(tester, testValue.indexOf('g'));
      final Offset pPos = textOffsetToPosition(tester, testValue.indexOf('p'));

      // A double tap + drag should take precedence over parent drags.
      final TestGesture gesture = await tester.startGesture(gPos);
      await tester.pump();
      await gesture.up();
      await tester.pump();
      await gesture.down(gPos);
      await tester.pumpAndSettle();
      await gesture.moveTo(pPos);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      final TextEditingValue currentValue =
          tester.state<EditableTextState>(find.byType(EditableText)).textEditingValue;
      expect(
        currentValue.selection,
        TextSelection(baseOffset: testValue.indexOf('g'), extentOffset: testValue.indexOf('p') + 3),
      );

      expect(pageController.page, isNotNull);
      expect(pageController.page, 0.0);
      // A horizontal drag directly on the SelectableText should move the page
      // view to the next page.
      final Rect selectableTextRect = tester.getRect(find.byType(SelectableText));
      await tester.dragFrom(
        selectableTextRect.centerRight - const Offset(0.1, 0.0),
        const Offset(-500.0, 0.0),
      );
      await tester.pumpAndSettle();
      expect(pageController.page, isNotNull);
      expect(pageController.page, 1.0);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'long press tap cannot initiate a double tap on macOS',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Center(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      await tester.longPressAt(selectableTextStart + const Offset(50.0, 5.0));
      await tester.pump(const Duration(milliseconds: 50));

      // Hide the toolbar so it doesn't interfere with taps on the text.
      final EditableTextState editableTextState = tester.state<EditableTextState>(
        find.byType(EditableText),
      );
      editableTextState.hideToolbar();
      await tester.pumpAndSettle();

      await tester.tapAt(selectableTextStart + const Offset(50.0, 5.0));
      await tester.pump();

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      // Move the cursor to the tapped position.
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 4, affinity: TextAffinity.upstream),
      );

      expect(find.byType(CupertinoButton), findsNothing);
    },
    variant: TargetPlatformVariant.only(TargetPlatform.macOS),
  );

  testWidgets(
    'long press tap cannot initiate a double tap on iOS',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Center(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      await tester.longPressAt(selectableTextStart + const Offset(50.0, 5.0));
      await tester.pump(const Duration(milliseconds: 50));

      // Hide the toolbar so it doesn't interfere with taps on the text.
      final EditableTextState editableTextState = tester.state<EditableTextState>(
        find.byType(EditableText),
      );
      editableTextState.hideToolbar();
      await tester.pumpAndSettle();

      await tester.tapAt(selectableTextStart + const Offset(50.0, 5.0));
      await tester.pump();

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      // Move the cursor to the edge of the same word and toggle the toolbar.
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));

      expect(find.byType(CupertinoButton), findsNWidgets(4));
    },
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );

  testWidgets(
    'long press drag extends the selection to the word under the drag and shows toolbar on lift on non-Apple platforms',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Center(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
          ),
        ),
      );

      final TestGesture gesture = await tester.startGesture(textOffsetToPosition(tester, 18));
      await tester.pump(const Duration(milliseconds: 500));

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      // Long press selects the word at the long presses position.
      expect(controller.selection, const TextSelection(baseOffset: 13, extentOffset: 23));
      // Cursor move doesn't trigger a toolbar initially.
      expect(find.byType(TextButton), findsNothing);

      await gesture.moveBy(const Offset(100, 0));
      await tester.pump();

      // The selection is now moved with the drag.
      expect(controller.selection, const TextSelection(baseOffset: 13, extentOffset: 35));
      // Still no toolbar.
      expect(find.byType(TextButton), findsNothing);

      // The selection is moved on a backwards drag.
      await gesture.moveBy(const Offset(-200, 0));
      await tester.pump();

      // The selection is now moved with the drag.
      expect(controller.selection, const TextSelection(baseOffset: 23, extentOffset: 8));
      // Still no toolbar.
      expect(find.byType(TextButton), findsNothing);

      await gesture.moveBy(const Offset(-100, 0));
      await tester.pump();

      // The selection is now moved with the drag.
      expect(controller.selection, const TextSelection(baseOffset: 23, extentOffset: 0));
      // Still no toolbar.
      expect(find.byType(TextButton), findsNothing);

      await gesture.up();
      await tester.pumpAndSettle();

      // The selection isn't affected by the gesture lift.
      expect(controller.selection, const TextSelection(baseOffset: 23, extentOffset: 0));

      expectMaterialSelectionToolbar();
    },
    variant: TargetPlatformVariant.all(
      excluding: <TargetPlatform>{TargetPlatform.iOS, TargetPlatform.macOS},
    ),
  );

  testWidgets(
    'long press drag extends the selection to the word under the drag and shows toolbar on lift (iOS)',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Center(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
          ),
        ),
      );

      final TestGesture gesture = await tester.startGesture(textOffsetToPosition(tester, 18));
      await tester.pump(const Duration(milliseconds: 500));

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      // The long pressed word is selected.
      expect(controller.selection, const TextSelection(baseOffset: 13, extentOffset: 23));
      // Word select doesn't trigger a toolbar initially.
      expect(find.byType(CupertinoButton), findsNothing);

      await gesture.moveBy(const Offset(100, 0));
      await tester.pump();

      // The selection is now moved with the drag.
      expect(controller.selection, const TextSelection(baseOffset: 13, extentOffset: 35));
      // Still no toolbar.
      expect(find.byType(CupertinoButton), findsNothing);

      // The selection is moved with a backwards drag.
      await gesture.moveBy(const Offset(-200, 0));
      await tester.pump();

      // The selection is now moved with the drag.
      expect(controller.selection, const TextSelection(baseOffset: 23, extentOffset: 8));
      // Still no toolbar.
      expect(find.byType(CupertinoButton), findsNothing);

      // The selection is moved with a backwards drag.
      await gesture.moveBy(const Offset(-100, 0));
      await tester.pump();

      // The selection is now moved with the drag.
      expect(controller.selection, const TextSelection(baseOffset: 23, extentOffset: 0));
      // Still no toolbar.
      expect(find.byType(CupertinoButton), findsNothing);

      await gesture.up();
      await tester.pump();

      // The selection isn't affected by the gesture lift.
      expect(controller.selection, const TextSelection(baseOffset: 23, extentOffset: 0));
      // The toolbar now shows up.
      expectCupertinoSelectionToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'long press drag moves the cursor under the drag and shows toolbar on lift (macOS)',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Center(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      final TestGesture gesture = await tester.startGesture(
        selectableTextStart + const Offset(50.0, 5.0),
      );
      await tester.pump(const Duration(milliseconds: 500));

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      // The long pressed word is selected.
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));
      // Cursor move doesn't trigger a toolbar initially.
      expect(find.byType(CupertinoButton), findsNothing);

      await gesture.moveBy(const Offset(50, 0));
      await tester.pump();

      // The selection position is now moved with the drag.
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 8));
      // Still no toolbar.
      expect(find.byType(CupertinoButton), findsNothing);

      await gesture.moveBy(const Offset(50, 0));
      await tester.pump();

      // The selection position is now moved with the drag.
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 12));
      // Still no toolbar.
      expect(find.byType(CupertinoButton), findsNothing);

      await gesture.up();
      await tester.pump();

      // The selection isn't affected by the gesture lift.
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 12));
      // The toolbar now shows up.
      expectCupertinoSelectionToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.macOS}),
  );

  testWidgets(
    'long press drag can edge scroll when inside a scrollable',
    (WidgetTester tester) async {
      // This is a regression test for https://github.com/flutter/flutter/issues/129590.
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: SizedBox(
                width: 300.0,
                child: SingleChildScrollView(
                  scrollDirection: Axis.horizontal,
                  child: SelectableText(
                    'Atwater Peel Sherbrooke Bonaventure Angrignon Peel Côte-des-Neiges ' * 2,
                    maxLines: 1,
                  ),
                ),
              ),
            ),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      final TestGesture gesture = await tester.startGesture(
        selectableTextStart + const Offset(200.0, 0.0),
      );
      await tester.pump(kLongPressTimeout);

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      expect(controller.selection, const TextSelection(baseOffset: 13, extentOffset: 23));

      await gesture.moveBy(const Offset(100, 0));
      // To the edge of the screen basically.
      await tester.pump();
      expect(controller.selection, const TextSelection(baseOffset: 13, extentOffset: 23));
      // Keep moving out.
      await gesture.moveBy(const Offset(100, 0));
      await tester.pump();
      expect(controller.selection, const TextSelection(baseOffset: 13, extentOffset: 35));
      await gesture.moveBy(const Offset(1600, 0));
      await tester.pump();
      expect(controller.selection, const TextSelection(baseOffset: 13, extentOffset: 134));

      await gesture.up();
      await tester.pumpAndSettle();

      // The selection isn't affected by the gesture lift.
      expect(controller.selection, const TextSelection(baseOffset: 13, extentOffset: 134));
      // The toolbar shows up.
      if (defaultTargetPlatform == TargetPlatform.iOS) {
        expectCupertinoSelectionToolbar();
      } else {
        expectMaterialSelectionToolbar();
      }

      final RenderEditable renderEditable = findRenderEditable(tester);
      final List<TextSelectionPoint> endpoints = globalize(
        renderEditable.getEndpointsForSelection(controller.selection),
        renderEditable,
      );
      expect(endpoints.isNotEmpty, isTrue);
      expect(endpoints.length, 2);
      expect(endpoints[0].point.dx, isNegative);
      expect(endpoints[1].point.dx, isPositive);
    },
    // TODO(Renzo-Olivares): Add in TargetPlatform.android in the line below when
    // we fix edge scrolling in a Scrollable https://github.com/flutter/flutter/issues/64059.
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'Desktop mouse drag can edge scroll when inside a horizontal scrollable',
    (WidgetTester tester) async {
      // This is a regression test for https://github.com/flutter/flutter/issues/129590.
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: SizedBox(
                width: 300.0,
                child: SingleChildScrollView(
                  scrollDirection: Axis.horizontal,
                  child: SelectableText(
                    'Atwater Peel Sherbrooke Bonaventure Angrignon Peel Côte-des-Neiges ' * 2,
                    maxLines: 1,
                  ),
                ),
              ),
            ),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      final TestGesture gesture = await tester.startGesture(
        selectableTextStart + const Offset(200.0, 0.0),
      );
      await tester.pump();

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      await gesture.moveBy(const Offset(100, 0));
      // To the edge of the screen basically.
      await tester.pump();
      expect(controller.selection, const TextSelection(baseOffset: 14, extentOffset: 21));
      // Keep moving out.
      await gesture.moveBy(const Offset(100, 0));
      await tester.pump();
      expect(controller.selection, const TextSelection(baseOffset: 14, extentOffset: 28));
      await gesture.moveBy(const Offset(1600, 0));
      await tester.pump();
      expect(controller.selection, const TextSelection(baseOffset: 14, extentOffset: 134));

      await gesture.up();
      await tester.pumpAndSettle();

      // The selection isn't affected by the gesture lift.
      expect(controller.selection, const TextSelection(baseOffset: 14, extentOffset: 134));

      final RenderEditable renderEditable = findRenderEditable(tester);
      final List<TextSelectionPoint> endpoints = globalize(
        renderEditable.getEndpointsForSelection(controller.selection),
        renderEditable,
      );
      expect(endpoints.isNotEmpty, isTrue);
      expect(endpoints.length, 2);
      expect(endpoints[0].point.dx, isNegative);
      expect(endpoints[1].point.dx, isPositive);
    },
    variant: TargetPlatformVariant.desktop(),
  );

  testWidgets(
    'long press drag can edge scroll',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: SelectableText(
                'Atwater Peel Sherbrooke Bonaventure Angrignon Peel Côte-des-Neiges ' * 2,
                maxLines: 1,
              ),
            ),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      final TestGesture gesture = await tester.startGesture(
        selectableTextStart + const Offset(300, 5),
      );
      await tester.pump(kLongPressTimeout);

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      expect(controller.selection, const TextSelection(baseOffset: 13, extentOffset: 23));

      await gesture.moveBy(const Offset(300, 0));
      // To the edge of the screen basically.
      await tester.pump();
      expect(controller.selection, const TextSelection(baseOffset: 13, extentOffset: 45));
      // Keep moving out.
      await gesture.moveBy(const Offset(300, 0));
      await tester.pump();
      expect(controller.selection, const TextSelection(baseOffset: 13, extentOffset: 66));
      await gesture.moveBy(const Offset(400, 0));
      await tester.pump();
      expect(controller.selection, const TextSelection(baseOffset: 13, extentOffset: 102));

      await gesture.moveBy(const Offset(700, 0));
      await tester.pump();
      expect(controller.selection, const TextSelection(baseOffset: 13, extentOffset: 134));

      await gesture.up();
      await tester.pumpAndSettle();

      // The selection isn't affected by the gesture lift.
      expect(controller.selection, const TextSelection(baseOffset: 13, extentOffset: 134));
      // The toolbar shows up.
      if (defaultTargetPlatform == TargetPlatform.iOS) {
        expectCupertinoSelectionToolbar();
      } else {
        expectMaterialSelectionToolbar();
      }

      // Find the selection handle fade transition after the start handle has been
      // hidden because it is out of view.
      final List<FadeTransition> transitionsAfter =
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

      expect(transitionsAfter.length, 2);

      final FadeTransition startHandleAfter = transitionsAfter[0];
      final FadeTransition endHandleAfter = transitionsAfter[1];

      expect(startHandleAfter.opacity.value, 0.0);
      expect(endHandleAfter.opacity.value, 1.0);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.android,
    }),
  );

  testWidgets(
    'long tap still selects after a double tap select (iOS)',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Center(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      await tester.tapAt(selectableTextStart + const Offset(150.0, 5.0));
      await tester.pump(const Duration(milliseconds: 50));

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      // First tap moved the cursor to the beginning of the second word.
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 12, affinity: TextAffinity.upstream),
      );
      await tester.tapAt(selectableTextStart + const Offset(150.0, 5.0));
      await tester.pump(const Duration(milliseconds: 500));

      await tester.longPressAt(selectableTextStart + const Offset(100.0, 5.0));
      await tester.pump();

      // Selected the "word" where the tap happened, which is the first space.
      // Because the "word" is a whitespace, the selection will shift to the
      // previous "word" that is not a whitespace.
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));

      expectCupertinoSelectionToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'long tap still selects after a double tap select (macOS)',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Center(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      await tester.tapAt(selectableTextStart + const Offset(150.0, 5.0));
      await tester.pump(const Duration(milliseconds: 50));

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      // First tap moves the cursor to the tapped position.
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 11, affinity: TextAffinity.upstream),
      );
      await tester.tapAt(selectableTextStart + const Offset(150.0, 5.0));
      await tester.pump(const Duration(milliseconds: 500));

      await tester.longPressAt(selectableTextStart + const Offset(100.0, 5.0));
      await tester.pump();

      // Selected the "word" where the tap happened, which is the first space.
      expect(controller.selection, const TextSelection(baseOffset: 7, extentOffset: 8));

      // The toolbar shows up.
      expectCupertinoSelectionToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.macOS}),
  );
  //convert
  testWidgets(
    'double tap after a long tap is not affected',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Center(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
          ),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      await tester.longPressAt(selectableTextStart + const Offset(50.0, 5.0));
      await tester.pump(const Duration(milliseconds: 50));

      // Hide the toolbar so it doesn't interfere with taps on the text.
      final EditableTextState editableTextState = tester.state<EditableTextState>(
        find.byType(EditableText),
      );
      editableTextState.hideToolbar();
      await tester.pumpAndSettle();

      await tester.tapAt(selectableTextStart + const Offset(150.0, 5.0));
      await tester.pump(const Duration(milliseconds: 50));

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      // First tap moved the cursor.
      expect(
        controller.selection,
        TextSelection.collapsed(
          offset: defaultTargetPlatform == TargetPlatform.iOS ? 12 : 11,
          affinity: TextAffinity.upstream,
        ),
      );
      await tester.tapAt(selectableTextStart + const Offset(150.0, 5.0));
      await tester.pump();

      // Double tap selection.
      expect(controller.selection, const TextSelection(baseOffset: 8, extentOffset: 12));

      expectCupertinoSelectionToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'double tap chains work',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Center(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
          ),
        ),
      );
      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      await tester.tapAt(selectableTextStart + const Offset(50.0, 5.0));
      await tester.pump(const Duration(milliseconds: 50));

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      expect(
        controller.selection,
        TextSelection.collapsed(
          offset: defaultTargetPlatform == TargetPlatform.iOS ? 7 : 4,
          affinity: TextAffinity.upstream,
        ),
      );
      await tester.tapAt(selectableTextStart + const Offset(50.0, 5.0));
      await tester.pump(const Duration(milliseconds: 50));
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));
      expectCupertinoSelectionToolbar();

      // Double tap selecting the same word somewhere else is fine.
      await tester.pumpAndSettle(kDoubleTapTimeout);
      await tester.tapAt(selectableTextStart + const Offset(10.0, 5.0));
      await tester.pump(const Duration(milliseconds: 50));

      // First tap moved the cursor and toggled the toolbar.
      expect(find.byType(CupertinoButton), findsNothing);
      expect(
        controller.selection,
        defaultTargetPlatform == TargetPlatform.iOS
            ? const TextSelection(baseOffset: 0, extentOffset: 7)
            : const TextSelection.collapsed(offset: 1, affinity: TextAffinity.upstream),
      );
      await tester.tapAt(selectableTextStart + const Offset(10.0, 5.0));
      await tester.pump(const Duration(milliseconds: 50));
      // Second tap toggled the toolbar, and on macOS also selects the word at the tapped position.
      // On iOS the selection remains the same.
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 7));
      expectCupertinoSelectionToolbar();

      // Hide the toolbar so it doesn't interfere with taps on the text.
      final EditableTextState editableTextState = tester.state<EditableTextState>(
        find.byType(EditableText),
      );
      editableTextState.hideToolbar();
      await tester.pumpAndSettle();

      await tester.pumpAndSettle(kDoubleTapTimeout);
      await tester.tapAt(selectableTextStart + const Offset(150.0, 5.0));
      await tester.pump(const Duration(milliseconds: 50));
      // First tap moved the cursor.
      expect(
        controller.selection,
        TextSelection.collapsed(
          offset: defaultTargetPlatform == TargetPlatform.iOS ? 12 : 11,
          affinity: TextAffinity.upstream,
        ),
      );
      await tester.tapAt(selectableTextStart + const Offset(150.0, 5.0));
      await tester.pump(const Duration(milliseconds: 50));
      expect(controller.selection, const TextSelection(baseOffset: 8, extentOffset: 12));
      expectCupertinoSelectionToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('force press does not select a word on (android)', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
      ),
    );

    final Offset offset = tester.getTopLeft(find.byType(SelectableText)) + const Offset(150.0, 5.0);

    final int pointerValue = tester.nextPointer;
    final TestGesture gesture = await tester.createGesture();
    await gesture.downWithCustomEvent(
      offset,
      PointerDownEvent(
        pointer: pointerValue,
        position: offset,
        pressure: 0.0,
        pressureMax: 6.0,
        pressureMin: 0.0,
      ),
    );
    await gesture.updateWithCustomEvent(
      PointerMoveEvent(
        pointer: pointerValue,
        position: offset + const Offset(150.0, 5.0),
        pressure: 0.5,
        pressureMin: 0,
      ),
    );

    final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
    final TextEditingController controller = editableTextWidget.controller;

    // We don't want this gesture to select any word on Android.
    expect(controller.selection, const TextSelection.collapsed(offset: -1));

    await gesture.up();
    await tester.pump();
    expect(find.byType(TextButton), findsNothing);
  });

  testWidgets(
    'force press selects word',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      final int pointerValue = tester.nextPointer;
      final Offset offset = selectableTextStart + const Offset(150.0, 5.0);
      final TestGesture gesture = await tester.createGesture();
      await gesture.downWithCustomEvent(
        offset,
        PointerDownEvent(
          pointer: pointerValue,
          position: offset,
          pressure: 0.0,
          pressureMax: 6.0,
          pressureMin: 0.0,
        ),
      );

      await gesture.updateWithCustomEvent(
        PointerMoveEvent(
          pointer: pointerValue,
          position: selectableTextStart + const Offset(150.0, 5.0),
          pressure: 0.5,
          pressureMin: 0,
        ),
      );

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      // We expect the force press to select a word at the given location.
      expect(controller.selection, const TextSelection(baseOffset: 8, extentOffset: 12));

      await gesture.up();
      await tester.pump();
      expectCupertinoSelectionToolbar();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'tap on non-force-press-supported devices work',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(child: SelectableText('Atwater Peel Sherbrooke Bonaventure')),
        ),
      );

      final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

      final int pointerValue = tester.nextPointer;
      final Offset offset = selectableTextStart + const Offset(150.0, 5.0);
      final TestGesture gesture = await tester.createGesture();
      await gesture.downWithCustomEvent(
        offset,
        PointerDownEvent(
          pointer: pointerValue,
          position: offset,
          // iPhone 6 and below report 0 across the board.
          pressure: 0,
          pressureMax: 0,
          pressureMin: 0,
        ),
      );

      await gesture.updateWithCustomEvent(
        PointerMoveEvent(
          pointer: pointerValue,
          position: selectableTextStart + const Offset(150.0, 5.0),
          pressure: 0.5,
          pressureMin: 0,
        ),
      );
      await gesture.up();

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
      final TextEditingController controller = editableTextWidget.controller;

      // The event should fallback to a normal tap and move the cursor.
      // Single taps selects the edge of the word.
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 12, affinity: TextAffinity.upstream),
      );

      await tester.pump();
      // Single taps shouldn't trigger the toolbar.
      expect(find.byType(CupertinoButton), findsNothing);

      // TODO(gspencergoog): Add in TargetPlatform.macOS in the line below when we
      // figure out what global state is leaking.
      // https://github.com/flutter/flutter/issues/43445
    },
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );

  testWidgets('default SelectableText debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

    const SelectableText('something').debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>['data: something']);
  });

  testWidgets('SelectableText implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

    // Not checking controller, inputFormatters, focusNode.
    const SelectableText(
      'something',
      style: TextStyle(color: Color(0xff00ff00)),
      textAlign: TextAlign.end,
      textDirection: TextDirection.ltr,
      textScaler: TextScaler.noScaling,
      autofocus: true,
      showCursor: true,
      minLines: 2,
      maxLines: 10,
      cursorWidth: 1.0,
      cursorHeight: 1.0,
      cursorRadius: Radius.zero,
      cursorColor: Color(0xff00ff00),
      scrollPhysics: ClampingScrollPhysics(),
      scrollBehavior: ScrollBehavior(),
      semanticsLabel: 'something else',
      enableInteractiveSelection: false,
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[
      'data: something',
      'semanticsLabel: something else',
      'style: TextStyle(inherit: true, color: ${const Color(0xff00ff00)})',
      'autofocus: true',
      'showCursor: true',
      'minLines: 2',
      'maxLines: 10',
      'textAlign: end',
      'textDirection: ltr',
      'textScaler: no scaling',
      'cursorWidth: 1.0',
      'cursorHeight: 1.0',
      'cursorRadius: Radius.circular(0.0)',
      'cursorColor: ${const Color(0xff00ff00)}',
      'selection disabled',
      'scrollPhysics: ClampingScrollPhysics',
      'scrollBehavior: ScrollBehavior',
    ]);
  });

  testWidgets('strut basic single line', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(platform: TargetPlatform.android, useMaterial3: false),
        home: const Material(child: Center(child: SelectableText('something'))),
      ),
    );

    expect(
      tester.getSize(find.byType(SelectableText)),
      // This is the height of the decoration (24) plus the metrics from the default
      // TextStyle of the theme (16).
      const Size(129.0, 14.0),
    );
  });

  testWidgets('strut TextStyle increases height', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(platform: TargetPlatform.android, useMaterial3: false),
        home: const Material(
          child: Center(child: SelectableText('something', style: TextStyle(fontSize: 20))),
        ),
      ),
    );

    expect(
      tester.getSize(find.byType(SelectableText)),
      // Strut should inherit the TextStyle.fontSize by default and produce the
      // same height as if it were disabled.
      const Size(183.0, 20.0),
    );

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(platform: TargetPlatform.android, useMaterial3: false),
        home: const Material(
          child: Center(
            child: SelectableText(
              'something',
              style: TextStyle(fontSize: 20),
              strutStyle: StrutStyle.disabled,
            ),
          ),
        ),
      ),
    );

    expect(
      tester.getSize(find.byType(SelectableText)),
      // The height here should match the previous version with strut enabled.
      const Size(183.0, 20.0),
    );
  });

  testWidgets('strut basic multi line', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(platform: TargetPlatform.android, useMaterial3: false),
        home: const Material(child: Center(child: SelectableText('something', maxLines: 6))),
      ),
    );

    expect(tester.getSize(find.byType(SelectableText)), const Size(129.0, 84.0));
  });

  testWidgets('strut no force small strut', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(platform: TargetPlatform.android, useMaterial3: false),
        home: const Material(
          child: Center(
            child: SelectableText(
              'something',
              maxLines: 6,
              strutStyle: StrutStyle(
                // The small strut is overtaken by the larger
                // TextStyle fontSize.
                fontSize: 5,
              ),
            ),
          ),
        ),
      ),
    );

    expect(
      tester.getSize(find.byType(SelectableText)),
      // When the strut's height is smaller than TextStyle's and forceStrutHeight
      // is disabled, then the TextStyle takes precedence. Should be the same height
      // as 'strut basic multi line'.
      const Size(129.0, 84.0),
    );
  });

  testWidgets('strut no force large strut', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(platform: TargetPlatform.android, useMaterial3: false),
        home: const Material(
          child: Center(
            child: SelectableText('something', maxLines: 6, strutStyle: StrutStyle(fontSize: 25)),
          ),
        ),
      ),
    );

    expect(
      tester.getSize(find.byType(SelectableText)),
      // When the strut's height is larger than TextStyle's and forceStrutHeight
      // is disabled, then the StrutStyle takes precedence.
      const Size(129.0, 150.0),
    );
  });

  testWidgets('strut height override', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(platform: TargetPlatform.android, useMaterial3: false),
        home: const Material(
          child: Center(
            child: SelectableText(
              'something',
              maxLines: 3,
              strutStyle: StrutStyle(fontSize: 8, forceStrutHeight: true),
            ),
          ),
        ),
      ),
    );

    expect(
      tester.getSize(find.byType(SelectableText)),
      // The smaller font size of strut make the field shorter than normal.
      const Size(129.0, 24.0),
    );
  });

  testWidgets('strut forces field taller', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(platform: TargetPlatform.android, useMaterial3: false),
        home: const Material(
          child: Center(
            child: SelectableText(
              'something',
              maxLines: 3,
              style: TextStyle(fontSize: 10),
              strutStyle: StrutStyle(fontSize: 18, forceStrutHeight: true),
            ),
          ),
        ),
      ),
    );

    expect(
      tester.getSize(find.byType(SelectableText)),
      // When the strut fontSize is larger than a provided TextStyle, the
      // strut's height takes precedence.
      const Size(93.0, 54.0),
    );
  });

  testWidgets('Caret center position', (WidgetTester tester) async {
    await tester.pumpWidget(
      overlay(
        child: const SizedBox(
          width: 300.0,
          child: SelectableText('abcd', textAlign: TextAlign.center),
        ),
      ),
    );

    final RenderEditable editable = findRenderEditable(tester);

    Offset topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 4)).topLeft,
    );
    expect(topLeft.dx, equals(427));

    topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 3)).topLeft,
    );
    expect(topLeft.dx, equals(413));

    topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 2)).topLeft,
    );
    expect(topLeft.dx, equals(399));

    topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 1)).topLeft,
    );
    expect(topLeft.dx, equals(385));
  });

  testWidgets('Caret indexes into trailing whitespace center align', (WidgetTester tester) async {
    await tester.pumpWidget(
      overlay(
        child: const SizedBox(
          width: 300.0,
          child: SelectableText('abcd    ', textAlign: TextAlign.center),
        ),
      ),
    );

    final RenderEditable editable = findRenderEditable(tester);

    Offset topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 7)).topLeft,
    );
    expect(topLeft.dx, equals(469));

    topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 8)).topLeft,
    );
    expect(topLeft.dx, equals(483));

    topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 4)).topLeft,
    );
    expect(topLeft.dx, equals(427));

    topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 3)).topLeft,
    );
    expect(topLeft.dx, equals(413));

    topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 2)).topLeft,
    );
    expect(topLeft.dx, equals(399));

    topLeft = editable.localToGlobal(
      editable.getLocalRectForCaret(const TextPosition(offset: 1)).topLeft,
    );
    expect(topLeft.dx, equals(385));
  });

  testWidgets('selection handles are rendered and not faded away', (WidgetTester tester) async {
    const String testText = 'lorem ipsum';
    await tester.pumpWidget(const MaterialApp(home: Material(child: SelectableText(testText))));

    final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
    final RenderEditable renderEditable = state.renderEditable;

    await tester.tapAt(const Offset(20, 10));
    renderEditable.selectWord(cause: SelectionChangedCause.longPress);
    await tester.pumpAndSettle();

    final List<FadeTransition> transitions =
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
    final FadeTransition left = transitions[0];
    final FadeTransition right = transitions[1];

    expect(left.opacity.value, equals(1.0));
    expect(right.opacity.value, equals(1.0));
  });

  testWidgets(
    'selection handles are rendered and not faded away',
    (WidgetTester tester) async {
      const String testText = 'lorem ipsum';

      await tester.pumpWidget(const MaterialApp(home: Material(child: SelectableText(testText))));

      final RenderEditable renderEditable =
          tester.state<EditableTextState>(find.byType(EditableText)).renderEditable;

      await tester.tapAt(const Offset(20, 10));
      renderEditable.selectWord(cause: SelectionChangedCause.longPress);
      await tester.pumpAndSettle();

      final List<Widget> transitions =
          find.byType(FadeTransition).evaluate().map((Element e) => e.widget).toList();
      expect(transitions.length, 2);
      final FadeTransition left = transitions[0] as FadeTransition;
      final FadeTransition right = transitions[1] as FadeTransition;

      expect(left.opacity.value, equals(1.0));
      expect(right.opacity.value, equals(1.0));
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('Long press shows handles and toolbar', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: SelectableText('abc def ghi'))),
    );

    // Long press at 'e' in 'def'.
    final Offset ePos = textOffsetToPosition(tester, 5);
    await tester.longPressAt(ePos);
    await tester.pumpAndSettle();

    final EditableTextState editableText = tester.state(find.byType(EditableText));
    expect(editableText.selectionOverlay!.handlesAreVisible, isTrue);
    expect(editableText.selectionOverlay!.toolbarIsVisible, isTrue);
  });

  testWidgets('Double tap shows handles and toolbar', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: SelectableText('abc def ghi'))),
    );

    // Double tap at 'e' in 'def'.
    final Offset ePos = textOffsetToPosition(tester, 5);
    await tester.tapAt(ePos);
    await tester.pump(const Duration(milliseconds: 50));
    await tester.tapAt(ePos);
    await tester.pump();

    final EditableTextState editableText = tester.state(find.byType(EditableText));
    expect(editableText.selectionOverlay!.handlesAreVisible, isTrue);
    expect(editableText.selectionOverlay!.toolbarIsVisible, isTrue);
  });

  testWidgets('Mouse tap does not show handles nor toolbar', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: SelectableText('abc def ghi'))),
    );

    // Long press to trigger the selectable text.
    final Offset ePos = textOffsetToPosition(tester, 5);
    final TestGesture gesture = await tester.startGesture(
      ePos,
      pointer: 7,
      kind: PointerDeviceKind.mouse,
    );
    await tester.pump();
    await gesture.up();
    await tester.pump();

    final EditableTextState editableText = tester.state(find.byType(EditableText));
    expect(editableText.selectionOverlay!.toolbarIsVisible, isFalse);
    expect(editableText.selectionOverlay!.handlesAreVisible, isFalse);
  });

  testWidgets('Mouse long press does not show handles nor toolbar', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: SelectableText('abc def ghi'))),
    );

    // Long press to trigger the selectable text.
    final Offset ePos = textOffsetToPosition(tester, 5);
    final TestGesture gesture = await tester.startGesture(
      ePos,
      pointer: 7,
      kind: PointerDeviceKind.mouse,
    );
    await tester.pump(const Duration(seconds: 2));
    await gesture.up();
    await tester.pump();

    final EditableTextState editableText = tester.state(find.byType(EditableText));
    expect(editableText.selectionOverlay!.toolbarIsVisible, isFalse);
    expect(editableText.selectionOverlay!.handlesAreVisible, isFalse);
  });

  testWidgets('Mouse double tap does not show handles nor toolbar', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: SelectableText('abc def ghi'))),
    );

    // Double tap to trigger the selectable text.
    final Offset selectableTextPos = tester.getCenter(find.byType(SelectableText));
    final TestGesture gesture = await tester.startGesture(
      selectableTextPos,
      pointer: 7,
      kind: PointerDeviceKind.mouse,
    );
    await tester.pump(const Duration(milliseconds: 50));
    await gesture.up();
    await tester.pump();
    await gesture.down(selectableTextPos);
    await tester.pump();
    await gesture.up();
    await tester.pump();

    final EditableTextState editableText = tester.state(find.byType(EditableText));
    expect(editableText.selectionOverlay!.toolbarIsVisible, isFalse);
    expect(editableText.selectionOverlay!.handlesAreVisible, isFalse);
  });

  testWidgets('text span with tap gesture recognizer works in selectable rich text', (
    WidgetTester tester,
  ) async {
    int spyTaps = 0;
    final TapGestureRecognizer spyRecognizer =
        TapGestureRecognizer()
          ..onTap = () {
            spyTaps += 1;
          };
    addTearDown(spyRecognizer.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: SelectableText.rich(
              TextSpan(
                children: <TextSpan>[
                  const TextSpan(text: 'Atwater '),
                  TextSpan(text: 'Peel', recognizer: spyRecognizer),
                  const TextSpan(text: ' Sherbrooke Bonaventure'),
                ],
              ),
            ),
          ),
        ),
      ),
    );
    expect(spyTaps, 0);
    final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

    await tester.tapAt(selectableTextStart + const Offset(150.0, 5.0));
    expect(spyTaps, 1);

    // Waits for a while to avoid double taps.
    await tester.pump(const Duration(seconds: 1));

    // Starts a long press.
    final TestGesture gesture = await tester.startGesture(
      selectableTextStart + const Offset(150.0, 5.0),
    );
    await tester.pump(const Duration(milliseconds: 500));
    await gesture.up();
    await tester.pump();
    final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);

    final TextEditingController controller = editableTextWidget.controller;
    // Long press still triggers selection.
    expect(controller.selection, const TextSelection(baseOffset: 8, extentOffset: 12));
    // Long press does not trigger gesture recognizer.
    expect(spyTaps, 1);
  });

  testWidgets('text span with long press gesture recognizer works in selectable rich text', (
    WidgetTester tester,
  ) async {
    int spyLongPress = 0;
    final LongPressGestureRecognizer spyRecognizer =
        LongPressGestureRecognizer()
          ..onLongPress = () {
            spyLongPress += 1;
          };
    addTearDown(spyRecognizer.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: SelectableText.rich(
              TextSpan(
                children: <TextSpan>[
                  const TextSpan(text: 'Atwater '),
                  TextSpan(text: 'Peel', recognizer: spyRecognizer),
                  const TextSpan(text: ' Sherbrooke Bonaventure'),
                ],
              ),
            ),
          ),
        ),
      ),
    );
    expect(spyLongPress, 0);
    final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText));

    await tester.tapAt(selectableTextStart + const Offset(150.0, 5.0));
    expect(spyLongPress, 0);

    // Waits for a while to avoid double taps.
    await tester.pump(const Duration(seconds: 1));

    // Starts a long press.
    final TestGesture gesture = await tester.startGesture(
      selectableTextStart + const Offset(150.0, 5.0),
    );
    await tester.pump(const Duration(milliseconds: 500));
    await gesture.up();
    await tester.pump();
    final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);

    final TextEditingController controller = editableTextWidget.controller;
    // Long press does not trigger selection if there is text span with long
    // press recognizer.
    expect(
      controller.selection,
      const TextSelection(baseOffset: 11, extentOffset: 11, affinity: TextAffinity.upstream),
    );
    // Long press triggers gesture recognizer.
    expect(spyLongPress, 1);
  });

  testWidgets('SelectableText changes mouse cursor when hovered', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: Center(child: SelectableText('test')))),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.text('test')));

    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );
  });

  testWidgets(
    'The handles show after pressing Select All',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(home: Material(child: SelectableText('abc def ghi'))),
      );

      // Long press at 'e' in 'def'.
      final Offset ePos = textOffsetToPosition(tester, 5);
      await tester.longPressAt(ePos);
      await tester.pumpAndSettle();

      expect(find.text('Select all'), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Paste'), findsNothing);
      expect(find.text('Cut'), findsNothing);
      EditableTextState editableText = tester.state(find.byType(EditableText));
      expect(editableText.selectionOverlay!.handlesAreVisible, isTrue);
      expect(editableText.selectionOverlay!.toolbarIsVisible, isTrue);

      await tester.tap(find.text('Select all'));
      await tester.pump();
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Select all'), findsNothing);
      expect(find.text('Paste'), findsNothing);
      expect(find.text('Cut'), findsNothing);
      editableText = tester.state(find.byType(EditableText));
      expect(editableText.selectionOverlay!.handlesAreVisible, isTrue);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.fuchsia,
    }),
  );

  testWidgets(
    'The Select All calls on selection changed',
    (WidgetTester tester) async {
      TextSelection? newSelection;
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: SelectableText(
              'abc def ghi',
              onSelectionChanged: (TextSelection selection, SelectionChangedCause? cause) {
                expect(newSelection, isNull);
                newSelection = selection;
              },
            ),
          ),
        ),
      );

      // Long press at 'e' in 'def'.
      final Offset ePos = textOffsetToPosition(tester, 5);
      await tester.longPressAt(ePos);
      await tester.pumpAndSettle();

      expect(newSelection!.baseOffset, 4);
      expect(newSelection!.extentOffset, 7);
      newSelection = null;

      await tester.tap(find.text('Select all'));
      await tester.pump();
      expect(newSelection!.baseOffset, 0);
      expect(newSelection!.extentOffset, 11);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.fuchsia,
    }),
  );

  testWidgets(
    'The Select All calls on selection changed with a mouse on windows and linux',
    (WidgetTester tester) async {
      const String string = 'abc def ghi';
      TextSelection? newSelection;
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: SelectableText(
              string,
              onSelectionChanged: (TextSelection selection, SelectionChangedCause? cause) {
                expect(newSelection, isNull);
                newSelection = selection;
              },
            ),
          ),
        ),
      );

      // Right-click on the 'e' in 'def'.
      final Offset ePos = textOffsetToPosition(tester, 5);
      final TestGesture gesture = await tester.startGesture(
        ePos,
        kind: PointerDeviceKind.mouse,
        buttons: kSecondaryMouseButton,
      );
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(newSelection!.isCollapsed, isTrue);
      expect(newSelection!.baseOffset, 5);
      newSelection = null;

      await tester.tap(find.text('Select all'));
      await tester.pump();
      expect(newSelection!.baseOffset, 0);
      expect(newSelection!.extentOffset, 11);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.windows,
      TargetPlatform.linux,
    }),
  );

  testWidgets('Does not show handles when updated from the web engine', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: SelectableText('abc def ghi'))),
    );

    // Interact with the selectable text to establish the input connection.
    final Offset topLeft = tester.getTopLeft(find.byType(EditableText));
    final TestGesture gesture = await tester.startGesture(
      topLeft + const Offset(0.0, 5.0),
      kind: PointerDeviceKind.mouse,
    );
    await tester.pump(const Duration(milliseconds: 50));
    await gesture.up();
    await tester.pumpAndSettle();

    final EditableTextState state = tester.state(find.byType(EditableText));
    expect(state.selectionOverlay!.handlesAreVisible, isFalse);
    expect(state.currentTextEditingValue.selection, const TextSelection.collapsed(offset: 0));

    if (kIsWeb) {
      tester.testTextInput.updateEditingValue(
        const TextEditingValue(selection: TextSelection(baseOffset: 2, extentOffset: 7)),
      );
      // Wait for all the `setState` calls to be flushed.
      await tester.pumpAndSettle();
      expect(
        state.currentTextEditingValue.selection,
        const TextSelection(baseOffset: 2, extentOffset: 7),
      );
      expect(state.selectionOverlay!.handlesAreVisible, isFalse);
    }
  });

  testWidgets('onSelectionChanged is called when selection changes', (WidgetTester tester) async {
    int onSelectionChangedCallCount = 0;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SelectableText(
            'abc def ghi',
            onSelectionChanged: (TextSelection selection, SelectionChangedCause? cause) {
              onSelectionChangedCallCount += 1;
            },
          ),
        ),
      ),
    );

    // Long press to select 'abc'.
    final Offset aLocation = textOffsetToPosition(tester, 1);
    await tester.longPressAt(aLocation);
    await tester.pump();
    expect(onSelectionChangedCallCount, equals(1));
    // Long press to select 'def'.
    await tester.longPressAt(textOffsetToPosition(tester, 5));
    await tester.pump();
    expect(onSelectionChangedCallCount, equals(2));
    // Tap on 'Select all' option to select the whole text.
    await tester.tap(find.text('Select all'));
    expect(onSelectionChangedCallCount, equals(3));
  });

  testWidgets(
    'selecting a space selects the previous word on mobile',
    (WidgetTester tester) async {
      TextSelection? selection;

      await tester.pumpWidget(
        MaterialApp(
          home: SelectableText(
            ' blah blah',
            onSelectionChanged: (TextSelection newSelection, SelectionChangedCause? cause) {
              selection = newSelection;
            },
          ),
        ),
      );

      expect(selection, isNull);

      // Put the cursor at the end of the field.
      await tester.tapAt(textOffsetToPosition(tester, 10));
      expect(selection, isNotNull);
      expect(selection!.baseOffset, 10);
      expect(selection!.extentOffset, 10);

      // Long press on the second space and the previous word is selected.
      await tester.longPressAt(textOffsetToPosition(tester, 5));
      await tester.pumpAndSettle();
      expect(selection, isNotNull);
      expect(selection!.baseOffset, 1);
      expect(selection!.extentOffset, 5);

      // Put the cursor at the end of the field.
      await tester.tapAt(textOffsetToPosition(tester, 10));
      expect(selection, isNotNull);
      expect(selection!.baseOffset, 10);
      expect(selection!.extentOffset, 10);

      // Long press on the first space and the space is selected because there is
      // no previous word.
      await tester.longPressAt(textOffsetToPosition(tester, 0));
      await tester.pumpAndSettle();
      expect(selection, isNotNull);
      expect(selection!.baseOffset, 0);
      expect(selection!.extentOffset, 1);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.android,
    }),
  );

  testWidgets(
    'selecting a space selects the space on non-mobile platforms',
    (WidgetTester tester) async {
      TextSelection? selection;

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: SelectableText(
                ' blah blah',
                onSelectionChanged: (TextSelection newSelection, SelectionChangedCause? cause) {
                  selection = newSelection;
                },
              ),
            ),
          ),
        ),
      );

      expect(selection, isNull);

      // Put the cursor at the end of the field.
      await tester.tapAt(textOffsetToPosition(tester, 10));
      expect(selection, isNotNull);
      expect(selection!.baseOffset, 10);
      expect(selection!.extentOffset, 10);

      // Double tapping the second space selects it.
      await tester.pump(const Duration(milliseconds: 500));
      await tester.tapAt(textOffsetToPosition(tester, 5));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, 5));
      await tester.pumpAndSettle();
      expect(selection, isNotNull);
      expect(selection!.baseOffset, 5);
      expect(selection!.extentOffset, 6);

      // Tap at the beginning of the text to hide the toolbar, then at the end to
      // move the cursor to the end. On some platforms, the context menu would
      // otherwise block a tap on the end of the field.
      await tester.tapAt(textOffsetToPosition(tester, 0));
      await tester.pumpAndSettle();
      await tester.tapAt(textOffsetToPosition(tester, 10));
      expect(selection, isNotNull);
      expect(selection!.baseOffset, 10);
      expect(selection!.extentOffset, 10);

      // Double tapping the first space selects it.
      await tester.pump(const Duration(milliseconds: 500));
      await tester.tapAt(textOffsetToPosition(tester, 0));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, 0));
      await tester.pumpAndSettle();
      expect(selection, isNotNull);
      expect(selection!.baseOffset, 0);
      expect(selection!.extentOffset, 1);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.macOS,
      TargetPlatform.windows,
      TargetPlatform.linux,
      TargetPlatform.fuchsia,
    }),
  );

  testWidgets(
    'double tapping a space selects the previous word on mobile',
    (WidgetTester tester) async {
      TextSelection? selection;

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(
            child: Center(
              child: SelectableText(
                ' blah blah  \n  blah',
                onSelectionChanged: (TextSelection newSelection, SelectionChangedCause? cause) {
                  selection = newSelection;
                },
              ),
            ),
          ),
        ),
      );

      expect(selection, isNull);

      // Put the cursor at the end of the field.
      await tester.tapAt(textOffsetToPosition(tester, 19));
      expect(selection, isNotNull);
      expect(selection!.baseOffset, 19);
      expect(selection!.extentOffset, 19);

      // Double tapping the second space selects the previous word.
      await tester.pump(const Duration(milliseconds: 500));
      await tester.tapAt(textOffsetToPosition(tester, 5));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, 5));
      await tester.pumpAndSettle();
      expect(selection, isNotNull);
      expect(selection!.baseOffset, 1);
      expect(selection!.extentOffset, 5);

      // Double tapping does the same thing for the first space.
      await tester.pump(const Duration(milliseconds: 500));
      await tester.tapAt(textOffsetToPosition(tester, 0));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, 0));
      await tester.pumpAndSettle();
      expect(selection, isNotNull);
      expect(selection!.baseOffset, 0);
      expect(selection!.extentOffset, 1);

      // Put the cursor at the end of the field.
      await tester.tapAt(textOffsetToPosition(tester, 19));
      expect(selection, isNotNull);
      expect(selection!.baseOffset, 19);
      expect(selection!.extentOffset, 19);

      // Double tapping the last space selects all previous contiguous spaces on
      // both lines and the previous word.
      await tester.pump(const Duration(milliseconds: 500));
      await tester.tapAt(textOffsetToPosition(tester, 14));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, 14));
      await tester.pumpAndSettle();
      expect(selection, isNotNull);
      expect(selection!.baseOffset, 6);
      expect(selection!.extentOffset, 14);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.android,
    }),
  );

  testWidgets('text selection style 1', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: const Material(
          child: Center(
            child: Column(
              children: <Widget>[
                SelectableText.rich(
                  TextSpan(
                    children: <TextSpan>[
                      TextSpan(text: 'Atwater Peel ', style: TextStyle(fontSize: 30.0)),
                      TextSpan(text: 'Sherbrooke Bonaventure ', style: TextStyle(fontSize: 15.0)),
                      TextSpan(text: 'hi wassup!', style: TextStyle(fontSize: 10.0)),
                    ],
                  ),
                  key: Key('field0'),
                  selectionHeightStyle: ui.BoxHeightStyle.includeLineSpacingTop,
                  selectionWidthStyle: ui.BoxWidthStyle.max,
                ),
              ],
            ),
          ),
        ),
      ),
    );

    final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
    final TextEditingController controller = editableTextWidget.controller;
    controller.selection = const TextSelection(baseOffset: 0, extentOffset: 46);
    await tester.pump();

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('selectable_text_golden.TextSelectionStyle.1.png'),
    );
  }, skip: impellerEnabled); // https://github.com/flutter/flutter/issues/143616

  testWidgets('text selection style 2', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: const Material(
          child: Center(
            child: Column(
              children: <Widget>[
                SelectableText.rich(
                  TextSpan(
                    children: <TextSpan>[
                      TextSpan(text: 'Atwater Peel ', style: TextStyle(fontSize: 30.0)),
                      TextSpan(text: 'Sherbrooke Bonaventure ', style: TextStyle(fontSize: 15.0)),
                      TextSpan(text: 'hi wassup!', style: TextStyle(fontSize: 10.0)),
                    ],
                  ),
                  key: Key('field0'),
                  selectionHeightStyle: ui.BoxHeightStyle.includeLineSpacingBottom,
                ),
              ],
            ),
          ),
        ),
      ),
    );

    final EditableText editableTextWidget = tester.widget(find.byType(EditableText).first);
    final TextEditingController controller = editableTextWidget.controller;
    controller.selection = const TextSelection(baseOffset: 0, extentOffset: 46);
    await tester.pump();

    await expectLater(
      find.byType(MaterialApp),
      matchesGoldenFile('selectable_text_golden.TextSelectionStyle.2.png'),
    );
  }, skip: impellerEnabled); // https://github.com/flutter/flutter/issues/143616

  testWidgets('keeps alive when has focus', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: DefaultTabController(
          length: 2,
          child: Scaffold(
            body: NestedScrollView(
              headerSliverBuilder: (BuildContext context, bool innerBoxIsScrolled) {
                return <Widget>[
                  SliverToBoxAdapter(
                    child: Container(
                      height: 200,
                      color: Colors.black12,
                      child: const Center(child: Text('Sliver 1')),
                    ),
                  ),
                  const SliverToBoxAdapter(
                    child: Center(
                      child: TabBar(
                        labelColor: Colors.black,
                        tabs: <Tab>[Tab(text: 'Sliver Tab 1'), Tab(text: 'Sliver Tab 2')],
                      ),
                    ),
                  ),
                ];
              },
              body: const TabBarView(
                children: <Widget>[
                  Padding(padding: EdgeInsets.only(top: 100.0), child: Text('Regular Text')),
                  Padding(
                    padding: EdgeInsets.only(top: 100.0),
                    child: SelectableText('Selectable Text'),
                  ),
                ],
              ),
            ),
          ),
        ),
      ),
    );

    // Without any selection, the offscreen widget is disposed and can't be
    // found, for both Text and SelectableText.
    expect(find.text('Regular Text', skipOffstage: false), findsOneWidget);
    expect(find.byType(SelectableText, skipOffstage: false), findsNothing);

    await tester.tap(find.text('Sliver Tab 2'));
    await tester.pumpAndSettle();
    expect(find.text('Regular Text', skipOffstage: false), findsNothing);
    expect(find.byType(SelectableText, skipOffstage: false), findsOneWidget);

    await tester.tap(find.text('Sliver Tab 1'));
    await tester.pumpAndSettle();
    expect(find.text('Regular Text', skipOffstage: false), findsOneWidget);
    expect(find.byType(SelectableText, skipOffstage: false), findsNothing);

    // Switch back to tab 2 and select some text in SelectableText.
    await tester.tap(find.text('Sliver Tab 2'));
    await tester.pumpAndSettle();
    expect(find.text('Regular Text', skipOffstage: false), findsNothing);
    expect(find.byType(SelectableText, skipOffstage: false), findsOneWidget);

    final EditableText editableText = tester.widget(find.byType(EditableText));
    expect(editableText.controller.selection.isValid, isFalse);
    await tester.tapAt(textOffsetToPosition(tester, 4));
    await tester.pump(const Duration(milliseconds: 50));
    await tester.tapAt(textOffsetToPosition(tester, 4));
    await tester.pumpAndSettle();
    expect(editableText.controller.selection.isValid, isTrue);
    expect(editableText.controller.selection.baseOffset, 0);
    expect(editableText.controller.selection.extentOffset, 'Selectable'.length);

    // Switch back to tab 1. The SelectableText remains because it is preserving
    // its selection.
    await tester.tap(find.text('Sliver Tab 1'));
    await tester.pumpAndSettle();
    expect(find.text('Regular Text', skipOffstage: false), findsOneWidget);
    expect(find.byType(SelectableText, skipOffstage: false), findsOneWidget);
  });

  group('magnifier', () {
    late ValueNotifier<MagnifierInfo> magnifierInfo;
    final Widget fakeMagnifier = Container(key: UniqueKey());

    testWidgets('Can drag handles to show, unshow, and update magnifier', (
      WidgetTester tester,
    ) async {
      const String testValue = 'abc def ghi';
      final SelectableText selectableText = SelectableText(
        testValue,
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
      );

      await tester.pumpWidget(overlay(child: selectableText));

      await skipPastScrollingAnimation(tester);

      // Double tap the 'e' to select 'def'.
      await tester.tapAt(textOffsetToPosition(tester, testValue.indexOf('e')));
      await tester.pump(const Duration(milliseconds: 30));
      await tester.tapAt(textOffsetToPosition(tester, testValue.indexOf('e')));
      await tester.pump(const Duration(milliseconds: 30));

      final TextSelection selection = TextSelection(
        baseOffset: testValue.indexOf('d'),
        extentOffset: testValue.indexOf('f'),
      );

      final RenderEditable renderEditable = findRenderEditable(tester);
      final List<TextSelectionPoint> endpoints = globalize(
        renderEditable.getEndpointsForSelection(selection),
        renderEditable,
      );

      // Drag the right handle 2 letters to the right.
      final Offset handlePos = endpoints.last.point + const Offset(1.0, 1.0);
      final TestGesture gesture = await tester.startGesture(handlePos, pointer: 7);

      Offset? firstDragGesturePosition;

      await gesture.moveTo(textOffsetToPosition(tester, testValue.length - 2));
      await tester.pump();

      expect(find.byKey(fakeMagnifier.key!), findsOneWidget);
      firstDragGesturePosition = magnifierInfo.value.globalGesturePosition;

      await gesture.moveTo(textOffsetToPosition(tester, testValue.length));
      await tester.pump();

      // Expect the position the magnifier gets to have moved.
      expect(firstDragGesturePosition, isNot(magnifierInfo.value.globalGesturePosition));

      await gesture.up();
      await tester.pump();

      expect(find.byKey(fakeMagnifier.key!), findsNothing);
    });
  });

  testWidgets('SelectableText text span style is merged with default text style', (
    WidgetTester tester,
  ) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/71389

    const TextStyle textStyle = TextStyle(color: Color(0xff00ff00), fontSize: 12.0);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: const SelectableText.rich(TextSpan(text: 'Abcd', style: textStyle)),
      ),
    );

    final EditableText editableText = tester.widget(find.byType(EditableText));
    expect(editableText.style.fontSize, textStyle.fontSize);
  });

  testWidgets('SelectableText text span style is merged with default text style', (
    WidgetTester tester,
  ) async {
    TextSelection? selection;
    int count = 0;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: SelectableText(
          'I love Flutter!',
          onSelectionChanged: (TextSelection s, _) {
            selection = s;
            count++;
          },
        ),
      ),
    );

    expect(selection, null);
    expect(count, 0);

    // Tap to put the cursor before the "F".
    const int index = 7;
    await tester.tapAt(textOffsetToPosition(tester, index));
    await tester.pump(const Duration(milliseconds: 500));
    expect(selection, const TextSelection.collapsed(offset: index));
    expect(count, 1); // The `onSelectionChanged` will be triggered one time.

    // Tap on the same location again.
    await tester.tapAt(textOffsetToPosition(tester, index));
    await tester.pump(const Duration(milliseconds: 50));
    expect(selection, const TextSelection.collapsed(offset: index));
    expect(count, 1); // The `onSelectionChanged` will not be triggered.
  });

  testWidgets("Off-screen selected text doesn't throw exception", (WidgetTester tester) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/123527

    TextSelection? selection;
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Builder(
            builder: (BuildContext context) {
              return Column(
                children: <Widget>[
                  Expanded(
                    child: ListView.builder(
                      itemCount: 100,
                      itemBuilder: (BuildContext context, int index) {
                        return SelectableText(
                          'I love Flutter! $index',
                          onSelectionChanged: (TextSelection s, _) {
                            selection = s;
                          },
                        );
                      },
                    ),
                  ),
                  TextButton(
                    onPressed: () {
                      Navigator.pop(context);
                    },
                    child: const Text('Pop'),
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );

    expect(selection, null);

    final Offset selectableTextStart = tester.getTopLeft(find.byType(SelectableText).first);
    final TestGesture gesture = await tester.startGesture(
      selectableTextStart + const Offset(50, 5),
    );
    await tester.pump(const Duration(milliseconds: 500));
    await gesture.up();

    expect(selection, isNotNull);

    await tester.drag(find.byType(ListView), const Offset(0.0, -3000.0));
    await tester.pumpAndSettle();

    await tester.tap(find.text('Pop'));
    await tester.pumpAndSettle();

    expect(tester.takeException(), isNull);
  });
}
