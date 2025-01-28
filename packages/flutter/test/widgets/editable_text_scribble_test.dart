// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import 'editable_text_utils.dart';

void main() {
  const TextStyle textStyle = TextStyle();
  const Color cursorColor = Color.fromARGB(0xFF, 0xFF, 0x00, 0x00);
  late TextEditingController controller;
  late FocusNode focusNode;

  setUp(() async {
    controller = TextEditingController();
    focusNode = FocusNode(debugLabel: 'EditableText Node');
  });

  tearDown(() {
    controller.dispose();
    focusNode.dispose();
  });

  testWidgets(
    'selection rects re-sent when refocused',
    (WidgetTester tester) async {
      final List<List<SelectionRect>> log = <List<SelectionRect>>[];
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.textInput, (
        MethodCall methodCall,
      ) async {
        if (methodCall.method == 'TextInput.setSelectionRects') {
          final List<dynamic> args = methodCall.arguments as List<dynamic>;
          final List<SelectionRect> selectionRects = <SelectionRect>[];
          for (final dynamic rect in args) {
            selectionRects.add(
              SelectionRect(
                position: (rect as List<dynamic>)[4] as int,
                bounds: Rect.fromLTWH(
                  rect[0] as double,
                  rect[1] as double,
                  rect[2] as double,
                  rect[3] as double,
                ),
              ),
            );
          }
          log.add(selectionRects);
        }
        return null;
      });

      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);
      controller.text = 'Text1';

      Future<void> pumpEditableText({
        double? width,
        double? height,
        TextAlign textAlign = TextAlign.start,
      }) async {
        await tester.pumpWidget(
          MediaQuery(
            data: const MediaQueryData(),
            child: Directionality(
              textDirection: TextDirection.ltr,
              child: Center(
                child: SizedBox(
                  width: width,
                  height: height,
                  child: EditableText(
                    controller: controller,
                    textAlign: textAlign,
                    scrollController: scrollController,
                    maxLines: null,
                    focusNode: focusNode,
                    cursorWidth: 0,
                    style: Typography.material2018().black.titleMedium!,
                    cursorColor: Colors.blue,
                    backgroundCursorColor: Colors.grey,
                  ),
                ),
              ),
            ),
          ),
        );
      }

      const List<SelectionRect> expectedRects = <SelectionRect>[
        SelectionRect(position: 0, bounds: Rect.fromLTRB(0.0, 0.0, 14.0, 14.0)),
        SelectionRect(position: 1, bounds: Rect.fromLTRB(14.0, 0.0, 28.0, 14.0)),
        SelectionRect(position: 2, bounds: Rect.fromLTRB(28.0, 0.0, 42.0, 14.0)),
        SelectionRect(position: 3, bounds: Rect.fromLTRB(42.0, 0.0, 56.0, 14.0)),
        SelectionRect(position: 4, bounds: Rect.fromLTRB(56.0, 0.0, 70.0, 14.0)),
      ];

      await pumpEditableText();
      expect(log, isEmpty);

      await tester.showKeyboard(find.byType(EditableText));
      // First update.
      expect(log.single, expectedRects);
      log.clear();

      await tester.pumpAndSettle();
      expect(log, isEmpty);

      focusNode.unfocus();
      await tester.pumpAndSettle();
      expect(log, isEmpty);

      focusNode.requestFocus();
      //await tester.showKeyboard(find.byType(EditableText));
      await tester.pumpAndSettle();
      // Should re-receive the same rects.
      expect(log.single, expectedRects);
      log.clear();

      // On web, we should rely on the browser's implementation of Scribble, so we will not send selection rects.
    },
    skip: kIsWeb, // [intended]
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'Selection changes during Scribble interaction should have the scribble cause',
    (WidgetTester tester) async {
      controller.text = 'Lorem ipsum dolor sit amet';
      late SelectionChangedCause selectionCause;

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            controller: controller,
            backgroundCursorColor: Colors.grey,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
            onSelectionChanged: (TextSelection selection, SelectionChangedCause? cause) {
              if (cause != null) {
                selectionCause = cause;
              }
            },
          ),
        ),
      );

      await tester.showKeyboard(find.byType(EditableText));

      // A normal selection update from the framework has 'keyboard' as the cause.
      tester.testTextInput.updateEditingValue(
        TextEditingValue(
          text: controller.text,
          selection: const TextSelection(baseOffset: 2, extentOffset: 3),
        ),
      );
      await tester.pumpAndSettle();

      expect(selectionCause, SelectionChangedCause.keyboard);

      // A selection update during a scribble interaction has 'scribble' as the cause.
      await tester.testTextInput.startScribbleInteraction();
      tester.testTextInput.updateEditingValue(
        TextEditingValue(
          text: controller.text,
          selection: const TextSelection(baseOffset: 3, extentOffset: 4),
        ),
      );
      await tester.pumpAndSettle();

      expect(selectionCause, SelectionChangedCause.stylusHandwriting);

      await tester.testTextInput.finishScribbleInteraction();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'Requests focus and changes the selection when onScribbleFocus is called',
    (WidgetTester tester) async {
      controller.text = 'Lorem ipsum dolor sit amet';
      late SelectionChangedCause selectionCause;

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            controller: controller,
            backgroundCursorColor: Colors.grey,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
            onSelectionChanged: (TextSelection selection, SelectionChangedCause? cause) {
              if (cause != null) {
                selectionCause = cause;
              }
            },
          ),
        ),
      );

      await tester.testTextInput.scribbleFocusElement(
        TextInput.scribbleClients.keys.first,
        Offset.zero,
      );

      expect(focusNode.hasFocus, true);
      expect(selectionCause, SelectionChangedCause.stylusHandwriting);

      // On web, we should rely on the browser's implementation of Scribble, so the selection changed cause
      // will never be SelectionChangedCause.stylusHandwriting.
    },
    skip: kIsWeb, // [intended]
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'Declares itself for Scribble interaction if the bounds overlap the scribble rect and the widget is touchable',
    (WidgetTester tester) async {
      controller.text = 'Lorem ipsum dolor sit amet';

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            controller: controller,
            backgroundCursorColor: Colors.grey,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
          ),
        ),
      );

      final List<dynamic> elementEntry = <dynamic>[
        TextInput.scribbleClients.keys.first,
        0.0,
        0.0,
        800.0,
        600.0,
      ];

      List<List<dynamic>> elements = await tester.testTextInput.scribbleRequestElementsInRect(
        const Rect.fromLTWH(0, 0, 1, 1),
      );
      expect(elements.first, containsAll(elementEntry));

      // Touch is outside the bounds of the widget.
      elements = await tester.testTextInput.scribbleRequestElementsInRect(
        const Rect.fromLTWH(-1, -1, 1, 1),
      );
      expect(elements.length, 0);

      // Widget is read only.
      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            readOnly: true,
            controller: controller,
            backgroundCursorColor: Colors.grey,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
          ),
        ),
      );

      elements = await tester.testTextInput.scribbleRequestElementsInRect(
        const Rect.fromLTWH(0, 0, 1, 1),
      );
      expect(elements.length, 0);

      // Widget is not touchable.
      await tester.pumpWidget(
        MaterialApp(
          home: Stack(
            children: <Widget>[
              EditableText(
                controller: controller,
                backgroundCursorColor: Colors.grey,
                focusNode: focusNode,
                style: textStyle,
                cursorColor: cursorColor,
                selectionControls: materialTextSelectionControls,
              ),
              Positioned(
                left: 0,
                top: 0,
                right: 0,
                bottom: 0,
                child: Container(color: Colors.black),
              ),
            ],
          ),
        ),
      );

      elements = await tester.testTextInput.scribbleRequestElementsInRect(
        const Rect.fromLTWH(0, 0, 1, 1),
      );
      expect(elements.length, 0);

      // Widget has scribble disabled.
      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            controller: controller,
            backgroundCursorColor: Colors.grey,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
            stylusHandwritingEnabled: false,
          ),
        ),
      );

      elements = await tester.testTextInput.scribbleRequestElementsInRect(
        const Rect.fromLTWH(0, 0, 1, 1),
      );
      expect(elements.length, 0);

      // On web, we should rely on the browser's implementation of Scribble, so the engine will
      // never request the scribble elements.
    },
    skip: kIsWeb, // [intended]
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'single line Scribble fields can show a horizontal placeholder',
    (WidgetTester tester) async {
      controller.text = 'Lorem ipsum dolor sit amet';

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            controller: controller,
            backgroundCursorColor: Colors.grey,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
          ),
        ),
      );

      await tester.showKeyboard(find.byType(EditableText));

      tester.testTextInput.updateEditingValue(
        TextEditingValue(
          text: controller.text,
          selection: const TextSelection(baseOffset: 5, extentOffset: 5),
        ),
      );
      await tester.pumpAndSettle();

      await tester.testTextInput.scribbleInsertPlaceholder();
      await tester.pumpAndSettle();

      TextSpan textSpan = findRenderEditable(tester).text! as TextSpan;
      expect(textSpan.children!.length, 3);
      expect((textSpan.children![0] as TextSpan).text, 'Lorem');
      expect(textSpan.children![1] is WidgetSpan, true);
      expect((textSpan.children![2] as TextSpan).text, ' ipsum dolor sit amet');

      await tester.testTextInput.scribbleRemovePlaceholder();
      await tester.pumpAndSettle();

      textSpan = findRenderEditable(tester).text! as TextSpan;
      expect(textSpan.children, null);
      expect(textSpan.text, 'Lorem ipsum dolor sit amet');

      // Widget has scribble disabled.
      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            controller: controller,
            backgroundCursorColor: Colors.grey,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
            stylusHandwritingEnabled: false,
          ),
        ),
      );

      await tester.showKeyboard(find.byType(EditableText));

      tester.testTextInput.updateEditingValue(
        TextEditingValue(
          text: controller.text,
          selection: const TextSelection(baseOffset: 5, extentOffset: 5),
        ),
      );
      await tester.pumpAndSettle();

      await tester.testTextInput.scribbleInsertPlaceholder();
      await tester.pumpAndSettle();

      textSpan = findRenderEditable(tester).text! as TextSpan;
      expect(textSpan.children, null);
      expect(textSpan.text, 'Lorem ipsum dolor sit amet');

      // On web, we should rely on the browser's implementation of Scribble, so the framework
      // will not handle placeholders.
    },
    skip: kIsWeb, // [intended]
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'multiline Scribble fields can show a vertical placeholder',
    (WidgetTester tester) async {
      controller.text = 'Lorem ipsum dolor sit amet';

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            controller: controller,
            backgroundCursorColor: Colors.grey,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
            maxLines: 2,
          ),
        ),
      );

      await tester.showKeyboard(find.byType(EditableText));

      tester.testTextInput.updateEditingValue(
        TextEditingValue(
          text: controller.text,
          selection: const TextSelection(baseOffset: 5, extentOffset: 5),
        ),
      );
      await tester.pumpAndSettle();

      await tester.testTextInput.scribbleInsertPlaceholder();
      await tester.pumpAndSettle();

      TextSpan textSpan = findRenderEditable(tester).text! as TextSpan;
      expect(textSpan.children!.length, 4);
      expect((textSpan.children![0] as TextSpan).text, 'Lorem');
      expect(textSpan.children![1] is WidgetSpan, true);
      expect(textSpan.children![2] is WidgetSpan, true);
      expect((textSpan.children![3] as TextSpan).text, ' ipsum dolor sit amet');

      await tester.testTextInput.scribbleRemovePlaceholder();
      await tester.pumpAndSettle();

      textSpan = findRenderEditable(tester).text! as TextSpan;
      expect(textSpan.children, null);
      expect(textSpan.text, 'Lorem ipsum dolor sit amet');

      // Widget has scribble disabled.
      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            controller: controller,
            backgroundCursorColor: Colors.grey,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionControls,
            maxLines: 2,
            stylusHandwritingEnabled: false,
          ),
        ),
      );

      await tester.showKeyboard(find.byType(EditableText));

      tester.testTextInput.updateEditingValue(
        TextEditingValue(
          text: controller.text,
          selection: const TextSelection(baseOffset: 5, extentOffset: 5),
        ),
      );
      await tester.pumpAndSettle();

      await tester.testTextInput.scribbleInsertPlaceholder();
      await tester.pumpAndSettle();

      textSpan = findRenderEditable(tester).text! as TextSpan;
      expect(textSpan.children, null);
      expect(textSpan.text, 'Lorem ipsum dolor sit amet');

      // On web, we should rely on the browser's implementation of Scribble, so the framework
      // will not handle placeholders.
    },
    skip: kIsWeb, // [intended]
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'selection rects are sent when they change',
    (WidgetTester tester) async {
      addTearDown(tester.view.reset);
      // Ensure selection rects are sent on iPhone (using SE 3rd gen size)
      tester.view.physicalSize = const Size(750.0, 1334.0);

      final List<List<SelectionRect>> log = <List<SelectionRect>>[];
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.textInput, (
        MethodCall methodCall,
      ) {
        if (methodCall.method == 'TextInput.setSelectionRects') {
          final List<dynamic> args = methodCall.arguments as List<dynamic>;
          final List<SelectionRect> selectionRects = <SelectionRect>[];
          for (final dynamic rect in args) {
            selectionRects.add(
              SelectionRect(
                position: (rect as List<dynamic>)[4] as int,
                bounds: Rect.fromLTWH(
                  rect[0] as double,
                  rect[1] as double,
                  rect[2] as double,
                  rect[3] as double,
                ),
              ),
            );
          }
          log.add(selectionRects);
        }
        return null;
      });

      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);
      controller.text = 'Text1';

      Future<void> pumpEditableText({
        double? width,
        double? height,
        TextAlign textAlign = TextAlign.start,
      }) async {
        await tester.pumpWidget(
          MediaQuery(
            data: const MediaQueryData(),
            child: Directionality(
              textDirection: TextDirection.ltr,
              child: Center(
                child: SizedBox(
                  width: width,
                  height: height,
                  child: EditableText(
                    controller: controller,
                    textAlign: textAlign,
                    scrollController: scrollController,
                    maxLines: null,
                    focusNode: focusNode,
                    cursorWidth: 0,
                    style: Typography.material2018().black.titleMedium!,
                    cursorColor: Colors.blue,
                    backgroundCursorColor: Colors.grey,
                  ),
                ),
              ),
            ),
          ),
        );
      }

      await pumpEditableText();
      expect(log, isEmpty);

      await tester.showKeyboard(find.byType(EditableText));
      // First update.
      expect(log.single, const <SelectionRect>[
        SelectionRect(position: 0, bounds: Rect.fromLTRB(0.0, 0.0, 14.0, 14.0)),
        SelectionRect(position: 1, bounds: Rect.fromLTRB(14.0, 0.0, 28.0, 14.0)),
        SelectionRect(position: 2, bounds: Rect.fromLTRB(28.0, 0.0, 42.0, 14.0)),
        SelectionRect(position: 3, bounds: Rect.fromLTRB(42.0, 0.0, 56.0, 14.0)),
        SelectionRect(position: 4, bounds: Rect.fromLTRB(56.0, 0.0, 70.0, 14.0)),
      ]);
      log.clear();

      await tester.pumpAndSettle();
      expect(log, isEmpty);

      await pumpEditableText();
      expect(log, isEmpty);

      // Change the width such that each character occupies a line.
      await pumpEditableText(width: 20);
      expect(log.single, const <SelectionRect>[
        SelectionRect(position: 0, bounds: Rect.fromLTRB(0.0, 0.0, 14.0, 14.0)),
        SelectionRect(position: 1, bounds: Rect.fromLTRB(0.0, 14.0, 14.0, 28.0)),
        SelectionRect(position: 2, bounds: Rect.fromLTRB(0.0, 28.0, 14.0, 42.0)),
        SelectionRect(position: 3, bounds: Rect.fromLTRB(0.0, 42.0, 14.0, 56.0)),
        SelectionRect(position: 4, bounds: Rect.fromLTRB(0.0, 56.0, 14.0, 70.0)),
      ]);
      log.clear();

      await tester.enterText(find.byType(EditableText), 'Text1👨‍👩‍👦');
      await tester.pump();
      expect(log.single, const <SelectionRect>[
        SelectionRect(position: 0, bounds: Rect.fromLTRB(0.0, 0.0, 14.0, 14.0)),
        SelectionRect(position: 1, bounds: Rect.fromLTRB(0.0, 14.0, 14.0, 28.0)),
        SelectionRect(position: 2, bounds: Rect.fromLTRB(0.0, 28.0, 14.0, 42.0)),
        SelectionRect(position: 3, bounds: Rect.fromLTRB(0.0, 42.0, 14.0, 56.0)),
        SelectionRect(position: 4, bounds: Rect.fromLTRB(0.0, 56.0, 14.0, 70.0)),
        SelectionRect(position: 5, bounds: Rect.fromLTRB(0.0, 70.0, 42.0, 84.0)),
      ]);
      log.clear();

      // The 4th line will be partially visible.
      await pumpEditableText(width: 20, height: 45);
      expect(log.single, const <SelectionRect>[
        SelectionRect(position: 0, bounds: Rect.fromLTRB(0.0, 0.0, 14.0, 14.0)),
        SelectionRect(position: 1, bounds: Rect.fromLTRB(0.0, 14.0, 14.0, 28.0)),
        SelectionRect(position: 2, bounds: Rect.fromLTRB(0.0, 28.0, 14.0, 42.0)),
        SelectionRect(position: 3, bounds: Rect.fromLTRB(0.0, 42.0, 14.0, 56.0)),
      ]);
      log.clear();

      await pumpEditableText(width: 20, height: 45, textAlign: TextAlign.right);
      // This is 1px off from being completely right-aligned. The 1px width is
      // reserved for caret.
      expect(log.single, const <SelectionRect>[
        SelectionRect(position: 0, bounds: Rect.fromLTRB(5.0, 0.0, 19.0, 14.0)),
        SelectionRect(position: 1, bounds: Rect.fromLTRB(5.0, 14.0, 19.0, 28.0)),
        SelectionRect(position: 2, bounds: Rect.fromLTRB(5.0, 28.0, 19.0, 42.0)),
        SelectionRect(position: 3, bounds: Rect.fromLTRB(5.0, 42.0, 19.0, 56.0)),
        // These 2 lines will be out of bounds.
        // SelectionRect(position: 4, bounds: Rect.fromLTRB(5.0, 56.0, 19.0, 70.0)),
        // SelectionRect(position: 5, bounds: Rect.fromLTRB(-23.0, 70.0, 19.0, 84.0)),
      ]);
      log.clear();

      expect(scrollController.offset, 0);

      // Scrolling also triggers update.
      scrollController.jumpTo(14);
      await tester.pumpAndSettle();
      expect(log.single, const <SelectionRect>[
        SelectionRect(position: 0, bounds: Rect.fromLTRB(5.0, -14.0, 19.0, 0.0)),
        SelectionRect(position: 1, bounds: Rect.fromLTRB(5.0, 0.0, 19.0, 14.0)),
        SelectionRect(position: 2, bounds: Rect.fromLTRB(5.0, 14.0, 19.0, 28.0)),
        SelectionRect(position: 3, bounds: Rect.fromLTRB(5.0, 28.0, 19.0, 42.0)),
        SelectionRect(position: 4, bounds: Rect.fromLTRB(5.0, 42.0, 19.0, 56.0)),
        // This line is skipped because it's below the bottom edge of the render
        // object.
        // SelectionRect(position: 5, bounds: Rect.fromLTRB(5.0, 56.0, 47.0, 70.0)),
      ]);
      log.clear();

      // On web, we should rely on the browser's implementation of Scribble, so we will not send selection rects.
    },
    skip: kIsWeb, // [intended]
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'selection rects are not sent if stylusHandwritingEnabled is false',
    (WidgetTester tester) async {
      final List<MethodCall> log = <MethodCall>[];
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.textInput, (
        MethodCall methodCall,
      ) async {
        log.add(methodCall);
        return null;
      });

      controller.text = 'Text1';

      await tester.pumpWidget(
        MediaQuery(
          data: const MediaQueryData(),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                EditableText(
                  key: ValueKey<String>(controller.text),
                  controller: controller,
                  focusNode: focusNode,
                  style: Typography.material2018().black.titleMedium!,
                  cursorColor: Colors.blue,
                  backgroundCursorColor: Colors.grey,
                  stylusHandwritingEnabled: false,
                ),
              ],
            ),
          ),
        ),
      );
      await tester.showKeyboard(find.byKey(ValueKey<String>(controller.text)));

      // There should be a new platform message updating the selection rects.
      expect(log.where((MethodCall m) => m.method == 'TextInput.setSelectionRects').length, 0);

      // On web, we should rely on the browser's implementation of Scribble, so we will not send selection rects.
    },
    skip: kIsWeb, // [intended]
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'selection rects sent even when character corners are outside of paintBounds',
    (WidgetTester tester) async {
      final List<List<SelectionRect>> log = <List<SelectionRect>>[];
      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.textInput, (
        MethodCall methodCall,
      ) {
        if (methodCall.method == 'TextInput.setSelectionRects') {
          final List<dynamic> args = methodCall.arguments as List<dynamic>;
          final List<SelectionRect> selectionRects = <SelectionRect>[];
          for (final dynamic rect in args) {
            selectionRects.add(
              SelectionRect(
                position: (rect as List<dynamic>)[4] as int,
                bounds: Rect.fromLTWH(
                  rect[0] as double,
                  rect[1] as double,
                  rect[2] as double,
                  rect[3] as double,
                ),
              ),
            );
          }
          log.add(selectionRects);
        }
        return null;
      });

      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);
      controller.text = 'Text1';

      final GlobalKey<EditableTextState> editableTextKey = GlobalKey();

      Future<void> pumpEditableText({
        double? width,
        double? height,
        TextAlign textAlign = TextAlign.start,
      }) async {
        await tester.pumpWidget(
          MediaQuery(
            data: const MediaQueryData(),
            child: Directionality(
              textDirection: TextDirection.ltr,
              child: Center(
                child: SizedBox(
                  width: width,
                  height: height,
                  child: EditableText(
                    controller: controller,
                    textAlign: textAlign,
                    scrollController: scrollController,
                    maxLines: null,
                    focusNode: focusNode,
                    cursorWidth: 0,
                    key: editableTextKey,
                    style: Typography.material2018().black.titleMedium!,
                    cursorColor: Colors.blue,
                    backgroundCursorColor: Colors.grey,
                  ),
                ),
              ),
            ),
          ),
        );
      }

      // Set height to 1 pixel less than full height.
      await pumpEditableText(height: 13);
      expect(log, isEmpty);

      // Scroll so that the top of each character is above the top of the renderEditable
      // and the bottom of each character is below the bottom of the renderEditable.
      final ViewportOffset offset = ViewportOffset.fixed(0.5);
      addTearDown(offset.dispose);
      editableTextKey.currentState!.renderEditable.offset = offset;

      await tester.showKeyboard(find.byType(EditableText));
      // We should get all the rects.
      expect(log.single, const <SelectionRect>[
        SelectionRect(position: 0, bounds: Rect.fromLTRB(0.0, -0.5, 14.0, 13.5)),
        SelectionRect(position: 1, bounds: Rect.fromLTRB(14.0, -0.5, 28.0, 13.5)),
        SelectionRect(position: 2, bounds: Rect.fromLTRB(28.0, -0.5, 42.0, 13.5)),
        SelectionRect(position: 3, bounds: Rect.fromLTRB(42.0, -0.5, 56.0, 13.5)),
        SelectionRect(position: 4, bounds: Rect.fromLTRB(56.0, -0.5, 70.0, 13.5)),
      ]);
      log.clear();

      // On web, we should rely on the browser's implementation of Scribble, so we will not send selection rects.
    },
    skip: kIsWeb, // [intended]
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  // Regression test for https://github.com/flutter/flutter/issues/159259.
  testWidgets(
    'showToolbar does nothing and returns false when already shown during Scribble selection',
    (WidgetTester tester) async {
      controller.text = 'Lorem ipsum dolor sit amet';
      final GlobalKey<EditableTextState> editableTextKey = GlobalKey();

      await tester.pumpWidget(
        MaterialApp(
          home: EditableText(
            key: editableTextKey,
            controller: controller,
            backgroundCursorColor: Colors.grey,
            focusNode: focusNode,
            style: textStyle,
            cursorColor: cursorColor,
            selectionControls: materialTextSelectionHandleControls,
            contextMenuBuilder: (BuildContext context, EditableTextState editableTextState) {
              return AdaptiveTextSelectionToolbar.editableText(
                editableTextState: editableTextState,
              );
            },
          ),
        ),
      );

      expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);

      await tester.showKeyboard(find.byType(EditableText));

      await tester.testTextInput.startScribbleInteraction();
      tester.testTextInput.updateEditingValue(
        TextEditingValue(
          text: controller.text,
          selection: const TextSelection(baseOffset: 3, extentOffset: 4),
        ),
      );
      await tester.pumpAndSettle();

      expect(find.byType(AdaptiveTextSelectionToolbar), findsNothing);

      expect(editableTextKey.currentState!.showToolbar(), isTrue);
      await tester.pumpAndSettle();

      expect(find.byType(AdaptiveTextSelectionToolbar), findsOneWidget);

      expect(editableTextKey.currentState!.showToolbar(), isFalse);
      await tester.pump();

      expect(find.byType(AdaptiveTextSelectionToolbar), findsOneWidget);

      await tester.pumpAndSettle();

      expect(find.byType(AdaptiveTextSelectionToolbar), findsOneWidget);

      await tester.testTextInput.finishScribbleInteraction();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
    skip: kIsWeb, // [intended]
  );
}
