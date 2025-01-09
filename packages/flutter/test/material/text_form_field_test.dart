// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import '../widgets/clipboard_utils.dart';
import '../widgets/editable_text_utils.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();
  final MockClipboard mockClipboard = MockClipboard();
  TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
    SystemChannels.platform,
    mockClipboard.handleMethodCall,
  );

  setUp(() async {
    // Fill the clipboard so that the Paste option is available in the text
    // selection menu.
    await Clipboard.setData(const ClipboardData(text: 'Clipboard data'));
  });

  testWidgets(
    'can use the desktop cut/copy/paste buttons on Mac',
    (WidgetTester tester) async {
      final TextEditingController controller = TextEditingController(text: 'blah1 blah2');
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextFormField(controller: controller)))),
      );

      // Initially, the menu is not shown and there is no selection.
      expect(find.byType(CupertinoButton), findsNothing);
      expect(controller.selection, const TextSelection(baseOffset: -1, extentOffset: -1));

      final Offset midBlah1 = textOffsetToPosition(tester, 2);

      // Right clicking shows the menu.
      final TestGesture gesture = await tester.startGesture(
        midBlah1,
        kind: PointerDeviceKind.mouse,
        buttons: kSecondaryMouseButton,
      );
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 5));
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);

      // Copy the first word.
      await tester.tap(find.text('Copy'));
      await tester.pumpAndSettle();
      expect(controller.text, 'blah1 blah2');
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 5));
      expect(find.byType(CupertinoButton), findsNothing);

      // Paste it at the end.
      await gesture.down(textOffsetToPosition(tester, controller.text.length));
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 11, affinity: TextAffinity.upstream),
      );
      expect(find.text('Cut'), findsNothing);
      expect(find.text('Copy'), findsNothing);
      expect(find.text('Paste'), findsOneWidget);
      await tester.tap(find.text('Paste'));
      await tester.pumpAndSettle();
      expect(controller.text, 'blah1 blah2blah1');
      expect(controller.selection, const TextSelection.collapsed(offset: 16));

      // Cut the first word.
      await gesture.down(midBlah1);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 5));
      await tester.tap(find.text('Cut'));
      await tester.pumpAndSettle();
      expect(controller.text, ' blah2blah1');
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 0));
      expect(find.byType(CupertinoButton), findsNothing);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.macOS}),
    skip: kIsWeb, // [intended] we don't supply the cut/copy/paste buttons on the web.
  );

  testWidgets(
    'can use the desktop cut/copy/paste buttons on Windows and Linux',
    (WidgetTester tester) async {
      final TextEditingController controller = TextEditingController(text: 'blah1 blah2');
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextFormField(controller: controller)))),
      );

      // Initially, the menu is not shown and there is no selection.
      expect(find.byType(CupertinoButton), findsNothing);
      expect(controller.selection, const TextSelection(baseOffset: -1, extentOffset: -1));

      final Offset midBlah1 = textOffsetToPosition(tester, 2);

      // Right clicking shows the menu.
      TestGesture gesture = await tester.startGesture(
        midBlah1,
        kind: PointerDeviceKind.mouse,
        buttons: kSecondaryMouseButton,
      );
      await tester.pump();
      await gesture.up();
      await gesture.removePointer();
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection.collapsed(offset: 2));
      expect(find.text('Cut'), findsNothing);
      expect(find.text('Copy'), findsNothing);
      expect(find.text('Paste'), findsOneWidget);
      expect(find.text('Select all'), findsOneWidget);

      // Double tap to select the first word, then right click to show the menu.
      final Offset startBlah1 = textOffsetToPosition(tester, 0);
      gesture = await tester.startGesture(startBlah1, kind: PointerDeviceKind.mouse);
      await tester.pump();
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 100));
      await gesture.down(startBlah1);
      await tester.pump();
      await gesture.up();
      await gesture.removePointer();
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 5));
      expect(find.text('Cut'), findsNothing);
      expect(find.text('Copy'), findsNothing);
      expect(find.text('Paste'), findsNothing);
      expect(find.text('Select all'), findsNothing);
      gesture = await tester.startGesture(
        midBlah1,
        kind: PointerDeviceKind.mouse,
        buttons: kSecondaryMouseButton,
      );
      await tester.pump();
      await gesture.up();
      await gesture.removePointer();
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 5));
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);

      // Copy the first word.
      await tester.tap(find.text('Copy'));
      await tester.pumpAndSettle();
      expect(controller.text, 'blah1 blah2');
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 5));
      expect(find.byType(CupertinoButton), findsNothing);

      // Paste it at the end.
      gesture = await tester.startGesture(
        textOffsetToPosition(tester, controller.text.length),
        kind: PointerDeviceKind.mouse,
      );
      await tester.pump();
      await gesture.up();
      await gesture.removePointer();
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 11, affinity: TextAffinity.upstream),
      );
      gesture = await tester.startGesture(
        textOffsetToPosition(tester, controller.text.length),
        kind: PointerDeviceKind.mouse,
        buttons: kSecondaryMouseButton,
      );
      await tester.pump();
      await gesture.up();
      await gesture.removePointer();
      await tester.pumpAndSettle();
      expect(
        controller.selection,
        const TextSelection.collapsed(offset: 11, affinity: TextAffinity.upstream),
      );
      expect(find.text('Cut'), findsNothing);
      expect(find.text('Copy'), findsNothing);
      expect(find.text('Paste'), findsOneWidget);
      await tester.tap(find.text('Paste'));
      await tester.pumpAndSettle();
      expect(controller.text, 'blah1 blah2blah1');
      expect(controller.selection, const TextSelection.collapsed(offset: 16));

      // Cut the first word.
      gesture = await tester.startGesture(midBlah1, kind: PointerDeviceKind.mouse);
      await tester.pump();
      await gesture.up();
      await tester.pump(const Duration(milliseconds: 100));
      await gesture.down(startBlah1);
      await tester.pump();
      await gesture.up();
      await gesture.removePointer();
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 5));
      expect(find.text('Cut'), findsNothing);
      expect(find.text('Copy'), findsNothing);
      expect(find.text('Paste'), findsNothing);
      expect(find.text('Select all'), findsNothing);
      gesture = await tester.startGesture(
        textOffsetToPosition(tester, controller.text.length),
        kind: PointerDeviceKind.mouse,
        buttons: kSecondaryMouseButton,
      );
      await tester.pump();
      await gesture.up();
      await gesture.removePointer();
      await tester.pumpAndSettle();
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 5));
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);
      await tester.tap(find.text('Cut'));
      await tester.pumpAndSettle();
      expect(controller.text, ' blah2blah1');
      expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 0));
      expect(find.byType(CupertinoButton), findsNothing);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.linux,
      TargetPlatform.windows,
    }),
    skip: kIsWeb, // [intended] we don't supply the cut/copy/paste buttons on the web.
  );

  testWidgets(
    '$SelectionOverlay is not leaking',
    (WidgetTester tester) async {
      final TextEditingController controller = TextEditingController(text: 'blah1 blah2');
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextField(controller: controller)))),
      );

      final Offset startBlah1 = textOffsetToPosition(tester, 0);
      await tester.tapAt(startBlah1);
      await tester.pump(const Duration(milliseconds: 100));
      await tester.tapAt(startBlah1);
      await tester.pumpAndSettle();
      await tester.pump();
    },
    skip: kIsWeb, // [intended] we don't supply the cut/copy/paste buttons on the web.
  );

  testWidgets(
    'the desktop cut/copy/paste buttons are disabled for read-only obscured form fields',
    (WidgetTester tester) async {
      final TextEditingController controller = TextEditingController(text: 'blah1 blah2');
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: TextFormField(readOnly: true, obscureText: true, controller: controller),
            ),
          ),
        ),
      );

      // Initially, the menu is not shown and there is no selection.
      expect(find.byType(CupertinoButton), findsNothing);
      const TextSelection invalidSelection = TextSelection(baseOffset: -1, extentOffset: -1);
      expect(controller.selection, invalidSelection);

      final Offset midBlah1 = textOffsetToPosition(tester, 2);

      // Right clicking shows the menu.
      final TestGesture gesture = await tester.startGesture(
        midBlah1,
        kind: PointerDeviceKind.mouse,
        buttons: kSecondaryMouseButton,
      );
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(controller.selection, invalidSelection);
      expect(find.text('Copy'), findsNothing);
      expect(find.text('Cut'), findsNothing);
      expect(find.text('Paste'), findsNothing);
      expect(find.byType(CupertinoButton), findsNothing);
    },
    variant: TargetPlatformVariant.desktop(),
    skip: kIsWeb, // [intended] we don't supply the cut/copy/paste buttons on the web.
  );

  testWidgets(
    'the desktop cut/copy buttons are disabled for obscured form fields',
    (WidgetTester tester) async {
      final TextEditingController controller = TextEditingController(text: 'blah1 blah2');
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(child: TextFormField(obscureText: true, controller: controller)),
          ),
        ),
      );

      // Initially, the menu is not shown and there is no selection.
      expect(find.byType(CupertinoButton), findsNothing);
      const TextSelection invalidSelection = TextSelection(baseOffset: -1, extentOffset: -1);
      expect(controller.selection, invalidSelection);

      final Offset midBlah1 = textOffsetToPosition(tester, 2);

      // Make a selection.
      await tester.tapAt(midBlah1);
      await tester.pump();
      await tester.sendKeyDownEvent(LogicalKeyboardKey.shift);
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shift);
      await tester.pump();
      expect(controller.selection, const TextSelection(baseOffset: 2, extentOffset: 0));

      // Right clicking shows the menu.
      final TestGesture gesture = await tester.startGesture(
        midBlah1,
        kind: PointerDeviceKind.mouse,
        buttons: kSecondaryMouseButton,
      );
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();
      expect(find.text('Copy'), findsNothing);
      expect(find.text('Cut'), findsNothing);
      expect(find.text('Paste'), findsOneWidget);
    },
    variant: TargetPlatformVariant.desktop(),
    skip: kIsWeb, // [intended] we don't supply the cut/copy/paste buttons on the web.
  );

  testWidgets('TextFormField accepts TextField.noMaxLength as value to maxLength parameter', (
    WidgetTester tester,
  ) async {
    bool asserted;
    try {
      TextFormField(maxLength: TextField.noMaxLength);
      asserted = false;
    } catch (e) {
      asserted = true;
    }
    expect(asserted, false);
  });

  testWidgets('Passes textAlign to underlying TextField', (WidgetTester tester) async {
    const TextAlign alignment = TextAlign.center;

    await tester.pumpWidget(
      MaterialApp(home: Material(child: Center(child: TextFormField(textAlign: alignment)))),
    );

    final Finder textFieldFinder = find.byType(TextField);
    expect(textFieldFinder, findsOneWidget);

    final TextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.textAlign, alignment);
  });

  testWidgets('Passes scrollPhysics to underlying TextField', (WidgetTester tester) async {
    const ScrollPhysics scrollPhysics = ScrollPhysics();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(child: Center(child: TextFormField(scrollPhysics: scrollPhysics))),
      ),
    );

    final Finder textFieldFinder = find.byType(TextField);
    expect(textFieldFinder, findsOneWidget);

    final TextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.scrollPhysics, scrollPhysics);
  });

  testWidgets('Passes textAlignVertical to underlying TextField', (WidgetTester tester) async {
    const TextAlignVertical textAlignVertical = TextAlignVertical.bottom;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(child: Center(child: TextFormField(textAlignVertical: textAlignVertical))),
      ),
    );

    final Finder textFieldFinder = find.byType(TextField);
    expect(textFieldFinder, findsOneWidget);

    final TextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.textAlignVertical, textAlignVertical);
  });

  testWidgets('Passes textInputAction to underlying TextField', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(child: Center(child: TextFormField(textInputAction: TextInputAction.next))),
      ),
    );

    final Finder textFieldFinder = find.byType(TextField);
    expect(textFieldFinder, findsOneWidget);

    final TextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.textInputAction, TextInputAction.next);
  });

  testWidgets('Passes onEditingComplete to underlying TextField', (WidgetTester tester) async {
    void onEditingComplete() {}

    await tester.pumpWidget(
      MaterialApp(
        home: Material(child: Center(child: TextFormField(onEditingComplete: onEditingComplete))),
      ),
    );

    final Finder textFieldFinder = find.byType(TextField);
    expect(textFieldFinder, findsOneWidget);

    final TextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.onEditingComplete, onEditingComplete);
  });

  testWidgets('Passes cursor attributes to underlying TextField', (WidgetTester tester) async {
    const double cursorWidth = 3.14;
    const double cursorHeight = 6.28;
    const Radius cursorRadius = Radius.circular(4);
    const Color cursorColor = Colors.purple;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: TextFormField(
              cursorWidth: cursorWidth,
              cursorHeight: cursorHeight,
              cursorRadius: cursorRadius,
              cursorColor: cursorColor,
            ),
          ),
        ),
      ),
    );

    final Finder textFieldFinder = find.byType(TextField);
    expect(textFieldFinder, findsOneWidget);

    final TextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.cursorWidth, cursorWidth);
    expect(textFieldWidget.cursorHeight, cursorHeight);
    expect(textFieldWidget.cursorRadius, cursorRadius);
    expect(textFieldWidget.cursorColor, cursorColor);
  });

  testWidgets('onFieldSubmit callbacks are called', (WidgetTester tester) async {
    bool called = false;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: TextFormField(
              onFieldSubmitted: (String value) {
                called = true;
              },
            ),
          ),
        ),
      ),
    );

    await tester.showKeyboard(find.byType(TextField));
    await tester.testTextInput.receiveAction(TextInputAction.done);
    await tester.pump();
    expect(called, true);
  });

  testWidgets('onChanged callbacks are called', (WidgetTester tester) async {
    late String value;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: TextFormField(
              onChanged: (String v) {
                value = v;
              },
            ),
          ),
        ),
      ),
    );

    await tester.enterText(find.byType(TextField), 'Soup');
    await tester.pump();
    expect(value, 'Soup');
  });

  testWidgets('autovalidateMode is passed to super', (WidgetTester tester) async {
    int validateCalled = 0;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: TextFormField(
              autovalidateMode: AutovalidateMode.always,
              validator: (String? value) {
                validateCalled++;
                return null;
              },
            ),
          ),
        ),
      ),
    );

    expect(validateCalled, 1);
    await tester.enterText(find.byType(TextField), 'a');
    await tester.pump();
    expect(validateCalled, 2);
  });

  testWidgets('validate is called if widget is enabled', (WidgetTester tester) async {
    int validateCalled = 0;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: TextFormField(
              enabled: true,
              autovalidateMode: AutovalidateMode.always,
              validator: (String? value) {
                validateCalled += 1;
                return null;
              },
            ),
          ),
        ),
      ),
    );

    expect(validateCalled, 1);
    await tester.enterText(find.byType(TextField), 'a');
    await tester.pump();
    expect(validateCalled, 2);
  });

  testWidgets('Disabled field hides helper and counter in M2', (WidgetTester tester) async {
    const String helperText = 'helper text';
    const String counterText = 'counter text';
    const String errorText = 'error text';
    Widget buildFrame(bool enabled, bool hasError) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Center(
            child: TextFormField(
              decoration: InputDecoration(
                labelText: 'label text',
                helperText: helperText,
                counterText: counterText,
                errorText: hasError ? errorText : null,
                enabled: enabled,
              ),
            ),
          ),
        ),
      );
    }

    // When enabled is true, the helper/error and counter are visible.
    await tester.pumpWidget(buildFrame(true, false));
    Text helperWidget = tester.widget(find.text(helperText));
    Text counterWidget = tester.widget(find.text(counterText));
    expect(helperWidget.style!.color, isNot(equals(Colors.transparent)));
    expect(counterWidget.style!.color, isNot(equals(Colors.transparent)));
    await tester.pumpWidget(buildFrame(true, true));
    counterWidget = tester.widget(find.text(counterText));
    Text errorWidget = tester.widget(find.text(errorText));
    expect(helperWidget.style!.color, isNot(equals(Colors.transparent)));
    expect(errorWidget.style!.color, isNot(equals(Colors.transparent)));

    // When enabled is false, the helper/error and counter are not visible.
    await tester.pumpWidget(buildFrame(false, false));
    helperWidget = tester.widget(find.text(helperText));
    counterWidget = tester.widget(find.text(counterText));
    expect(helperWidget.style!.color, equals(Colors.transparent));
    expect(counterWidget.style!.color, equals(Colors.transparent));
    await tester.pumpWidget(buildFrame(false, true));
    errorWidget = tester.widget(find.text(errorText));
    counterWidget = tester.widget(find.text(counterText));
    expect(counterWidget.style!.color, equals(Colors.transparent));
    expect(errorWidget.style!.color, equals(Colors.transparent));
  });

  testWidgets('passing a buildCounter shows returned widget', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: TextFormField(
              buildCounter: (
                BuildContext context, {
                int? currentLength,
                int? maxLength,
                bool? isFocused,
              }) {
                return Text('$currentLength of $maxLength');
              },
              maxLength: 10,
            ),
          ),
        ),
      ),
    );

    expect(find.text('0 of 10'), findsOneWidget);

    await tester.enterText(find.byType(TextField), '01234');
    await tester.pump();

    expect(find.text('5 of 10'), findsOneWidget);
  });

  testWidgets('readonly text form field will hide cursor by default', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(child: TextFormField(initialValue: 'readonly', readOnly: true)),
        ),
      ),
    );

    await tester.showKeyboard(find.byType(TextFormField));
    expect(tester.testTextInput.hasAnyClients, false);

    await tester.tap(find.byType(TextField));
    await tester.pump();
    expect(tester.testTextInput.hasAnyClients, false);

    await tester.longPress(find.byType(TextFormField));
    await tester.pump();

    // Context menu should not have paste.
    expect(find.text('Select all'), findsOneWidget);
    expect(find.text('Paste'), findsNothing);

    final EditableTextState editableTextState = tester.firstState(find.byType(EditableText));
    final RenderEditable renderEditable = editableTextState.renderEditable;

    // Make sure it does not paint caret for a period of time.
    await tester.pump(const Duration(milliseconds: 200));
    expect(renderEditable, paintsExactlyCountTimes(#drawRect, 0));

    await tester.pump(const Duration(milliseconds: 200));
    expect(renderEditable, paintsExactlyCountTimes(#drawRect, 0));

    await tester.pump(const Duration(milliseconds: 200));
    expect(renderEditable, paintsExactlyCountTimes(#drawRect, 0));
  }, skip: isBrowser); // [intended] We do not use Flutter-rendered context menu on the Web.

  testWidgets('onTap is called upon tap', (WidgetTester tester) async {
    int tapCount = 0;
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: TextFormField(
              onTap: () {
                tapCount += 1;
              },
            ),
          ),
        ),
      ),
    );

    expect(tapCount, 0);
    await tester.tap(find.byType(TextField));
    // Wait a bit so they're all single taps and not double taps.
    await tester.pump(const Duration(milliseconds: 300));
    await tester.tap(find.byType(TextField));
    await tester.pump(const Duration(milliseconds: 300));
    await tester.tap(find.byType(TextField));
    await tester.pump(const Duration(milliseconds: 300));
    expect(tapCount, 3);
  });

  testWidgets('onTapOutside is called upon tap outside', (WidgetTester tester) async {
    int tapOutsideCount = 0;
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: Column(
              children: <Widget>[
                const Text('Outside'),
                TextFormField(
                  autofocus: true,
                  onTapOutside: (PointerEvent event) {
                    tapOutsideCount += 1;
                  },
                ),
              ],
            ),
          ),
        ),
      ),
    );
    await tester.pump(); // Wait for autofocus to take effect.

    expect(tapOutsideCount, 0);
    await tester.tap(find.byType(TextFormField));
    await tester.tap(find.text('Outside'));
    await tester.tap(find.text('Outside'));
    await tester.tap(find.text('Outside'));
    expect(tapOutsideCount, 3);
  });

  // Regression test for https://github.com/flutter/flutter/issues/134341.
  testWidgets('onTapOutside is not called upon tap outside when field is not focused', (
    WidgetTester tester,
  ) async {
    int tapOutsideCount = 0;
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: Column(
              children: <Widget>[
                const Text('Outside'),
                TextFormField(
                  onTapOutside: (PointerEvent event) {
                    tapOutsideCount += 1;
                  },
                ),
              ],
            ),
          ),
        ),
      ),
    );
    await tester.pump();

    expect(tapOutsideCount, 0);
    await tester.tap(find.byType(TextFormField));
    await tester.tap(find.text('Outside'));
    await tester.tap(find.text('Outside'));
    await tester.tap(find.text('Outside'));
    expect(tapOutsideCount, 0);
  });

  // Regression test for https://github.com/flutter/flutter/issues/127597.
  testWidgets(
    'The second TextFormField is clicked, triggers the onTapOutside callback of the previous TextFormField',
    (WidgetTester tester) async {
      final GlobalKey keyA = GlobalKey();
      final GlobalKey keyB = GlobalKey();
      final GlobalKey keyC = GlobalKey();
      bool outsideClickA = false;
      bool outsideClickB = false;
      bool outsideClickC = false;
      await tester.pumpWidget(
        MaterialApp(
          home: Align(
            alignment: Alignment.topLeft,
            child: Column(
              children: <Widget>[
                const Text('Outside'),
                Material(
                  child: TextFormField(
                    key: keyA,
                    groupId: 'Group A',
                    onTapOutside: (PointerDownEvent event) {
                      outsideClickA = true;
                    },
                  ),
                ),
                Material(
                  child: TextFormField(
                    key: keyB,
                    groupId: 'Group B',
                    onTapOutside: (PointerDownEvent event) {
                      outsideClickB = true;
                    },
                  ),
                ),
                Material(
                  child: TextFormField(
                    key: keyC,
                    groupId: 'Group C',
                    onTapOutside: (PointerDownEvent event) {
                      outsideClickC = true;
                    },
                  ),
                ),
              ],
            ),
          ),
        ),
      );

      await tester.pump();

      Future<void> click(Finder finder) async {
        await tester.tap(finder);
        await tester.enterText(finder, 'Hello');
        await tester.pump();
      }

      expect(outsideClickA, false);
      expect(outsideClickB, false);
      expect(outsideClickC, false);

      await click(find.byKey(keyA));
      await tester.showKeyboard(find.byKey(keyA));
      await tester.idle();
      expect(outsideClickA, false);
      expect(outsideClickB, false);
      expect(outsideClickC, false);

      await click(find.byKey(keyB));
      expect(outsideClickA, true);
      expect(outsideClickB, false);
      expect(outsideClickC, false);

      await click(find.byKey(keyC));
      expect(outsideClickA, true);
      expect(outsideClickB, true);
      expect(outsideClickC, false);

      await tester.tap(find.text('Outside'));
      expect(outsideClickA, true);
      expect(outsideClickB, true);
      expect(outsideClickC, true);
    },
  );

  // Regression test for https://github.com/flutter/flutter/issues/54472.
  testWidgets('reset resets the text fields value to the initialValue', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(child: Center(child: TextFormField(initialValue: 'initialValue'))),
      ),
    );

    await tester.enterText(find.byType(TextFormField), 'changedValue');

    final FormFieldState<String> state = tester.state<FormFieldState<String>>(
      find.byType(TextFormField),
    );
    state.reset();

    expect(find.text('changedValue'), findsNothing);
    expect(find.text('initialValue'), findsOneWidget);
  });

  testWidgets('reset resets the text fields value to the controller initial value', (
    WidgetTester tester,
  ) async {
    final TextEditingController controller = TextEditingController(text: 'initialValue');
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(home: Material(child: Center(child: TextFormField(controller: controller)))),
    );

    await tester.enterText(find.byType(TextFormField), 'changedValue');

    final FormFieldState<String> state = tester.state<FormFieldState<String>>(
      find.byType(TextFormField),
    );
    state.reset();

    expect(find.text('changedValue'), findsNothing);
    expect(find.text('initialValue'), findsOneWidget);
  });

  // Regression test for https://github.com/flutter/flutter/issues/34847.
  testWidgets("didChange resets the text field's value to empty when passed null", (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(MaterialApp(home: Material(child: Center(child: TextFormField()))));

    await tester.enterText(find.byType(TextFormField), 'changedValue');
    await tester.pump();
    expect(find.text('changedValue'), findsOneWidget);

    final FormFieldState<String> state = tester.state<FormFieldState<String>>(
      find.byType(TextFormField),
    );
    state.didChange(null);

    expect(find.text('changedValue'), findsNothing);
    expect(find.text(''), findsOneWidget);
  });

  // Regression test for https://github.com/flutter/flutter/issues/34847.
  testWidgets("reset resets the text field's value to empty when initialValue is null", (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(MaterialApp(home: Material(child: Center(child: TextFormField()))));

    await tester.enterText(find.byType(TextFormField), 'changedValue');
    await tester.pump();
    expect(find.text('changedValue'), findsOneWidget);

    final FormFieldState<String> state = tester.state<FormFieldState<String>>(
      find.byType(TextFormField),
    );
    state.reset();

    expect(find.text('changedValue'), findsNothing);
    expect(find.text(''), findsOneWidget);
  });

  // Regression test for https://github.com/flutter/flutter/issues/54472.
  testWidgets('didChange changes text fields value', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(child: Center(child: TextFormField(initialValue: 'initialValue'))),
      ),
    );

    expect(find.text('initialValue'), findsOneWidget);

    final FormFieldState<String> state = tester.state<FormFieldState<String>>(
      find.byType(TextFormField),
    );
    state.didChange('changedValue');

    expect(find.text('initialValue'), findsNothing);
    expect(find.text('changedValue'), findsOneWidget);
  });

  testWidgets('onChanged callbacks value and FormFieldState.value are sync', (
    WidgetTester tester,
  ) async {
    bool called = false;

    late FormFieldState<String> state;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: TextFormField(
              onChanged: (String value) {
                called = true;
                expect(value, state.value);
              },
            ),
          ),
        ),
      ),
    );

    state = tester.state<FormFieldState<String>>(find.byType(TextFormField));

    await tester.enterText(find.byType(TextField), 'Soup');

    expect(called, true);
  });

  testWidgets('autofillHints is passed to super', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: TextFormField(autofillHints: const <String>[AutofillHints.countryName]),
          ),
        ),
      ),
    );

    final TextField widget = tester.widget(find.byType(TextField));
    expect(widget.autofillHints, equals(const <String>[AutofillHints.countryName]));
  });

  testWidgets('autovalidateMode is passed to super', (WidgetTester tester) async {
    int validateCalled = 0;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Scaffold(
            body: TextFormField(
              autovalidateMode: AutovalidateMode.onUserInteraction,
              validator: (String? value) {
                validateCalled++;
                return null;
              },
            ),
          ),
        ),
      ),
    );

    expect(validateCalled, 0);
    await tester.enterText(find.byType(TextField), 'a');
    await tester.pump();
    expect(validateCalled, 1);
  });

  testWidgets('textSelectionControls is passed to super', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Scaffold(body: TextFormField(selectionControls: materialTextSelectionControls)),
        ),
      ),
    );

    final TextField widget = tester.widget(find.byType(TextField));
    expect(widget.selectionControls, equals(materialTextSelectionControls));
  });

  testWidgets('TextFormField respects hintTextDirection', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Directionality(
            textDirection: TextDirection.rtl,
            child: TextFormField(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: 'Some Label',
                hintText: 'Some Hint',
                hintTextDirection: TextDirection.ltr,
              ),
            ),
          ),
        ),
      ),
    );

    final Finder hintTextFinder = find.text('Some Hint');

    final Text hintText = tester.firstWidget(hintTextFinder);
    expect(hintText.textDirection, TextDirection.ltr);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Directionality(
            textDirection: TextDirection.rtl,
            child: TextFormField(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: 'Some Label',
                hintText: 'Some Hint',
              ),
            ),
          ),
        ),
      ),
    );

    final BuildContext context = tester.element(hintTextFinder);
    final TextDirection textDirection = Directionality.of(context);
    expect(textDirection, TextDirection.rtl);
  });

  testWidgets('Passes scrollController to underlying TextField', (WidgetTester tester) async {
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(child: Center(child: TextFormField(scrollController: scrollController))),
      ),
    );

    final Finder textFieldFinder = find.byType(TextField);
    expect(textFieldFinder, findsOneWidget);

    final TextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.scrollController, scrollController);
  });

  testWidgets('TextFormField changes mouse cursor when hovered', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: MouseRegion(
            cursor: SystemMouseCursors.forbidden,
            child: TextFormField(mouseCursor: SystemMouseCursors.grab),
          ),
        ),
      ),
    );

    // Center, which is within the area
    final Offset center = tester.getCenter(find.byType(TextFormField));
    // Top left, which is also within the area
    final Offset edge = tester.getTopLeft(find.byType(TextFormField)) + const Offset(1, 1);

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: center);

    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.grab,
    );

    // Test default cursor
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: MouseRegion(cursor: SystemMouseCursors.forbidden, child: TextFormField()),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );
    await gesture.moveTo(edge);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );
    await gesture.moveTo(center);

    // Test default cursor when disabled
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: MouseRegion(
            cursor: SystemMouseCursors.forbidden,
            child: TextFormField(enabled: false),
          ),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );
    await gesture.moveTo(edge);
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );
    await gesture.moveTo(center);
  });

  // Regression test for https://github.com/flutter/flutter/issues/101587.
  testWidgets(
    'Right clicking menu behavior',
    (WidgetTester tester) async {
      final TextEditingController controller = TextEditingController(text: 'blah1 blah2');
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        MaterialApp(home: Material(child: Center(child: TextFormField(controller: controller)))),
      );

      // Initially, the menu is not shown and there is no selection.
      expect(find.byType(CupertinoButton), findsNothing);
      expect(controller.selection, const TextSelection(baseOffset: -1, extentOffset: -1));

      final Offset midBlah1 = textOffsetToPosition(tester, 2);
      final Offset midBlah2 = textOffsetToPosition(tester, 8);

      // Right click the second word.
      final TestGesture gesture = await tester.startGesture(
        midBlah2,
        kind: PointerDeviceKind.mouse,
        buttons: kSecondaryMouseButton,
      );
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();

      switch (defaultTargetPlatform) {
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          expect(controller.selection, const TextSelection(baseOffset: 6, extentOffset: 11));
          expect(find.text('Cut'), findsOneWidget);
          expect(find.text('Copy'), findsOneWidget);
          expect(find.text('Paste'), findsOneWidget);

        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          expect(controller.selection, const TextSelection.collapsed(offset: 8));
          expect(find.text('Cut'), findsNothing);
          expect(find.text('Copy'), findsNothing);
          expect(find.text('Paste'), findsOneWidget);
          expect(find.text('Select all'), findsOneWidget);
      }

      // Right click the first word.
      await gesture.down(midBlah1);
      await tester.pump();
      await gesture.up();
      await tester.pumpAndSettle();

      switch (defaultTargetPlatform) {
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          expect(controller.selection, const TextSelection(baseOffset: 0, extentOffset: 5));
          expect(find.text('Cut'), findsOneWidget);
          expect(find.text('Copy'), findsOneWidget);
          expect(find.text('Paste'), findsOneWidget);

        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          expect(controller.selection, const TextSelection.collapsed(offset: 8));
          expect(find.text('Cut'), findsNothing);
          expect(find.text('Copy'), findsNothing);
          expect(find.text('Paste'), findsNothing);
          expect(find.text('Select all'), findsNothing);
      }
    },
    variant: TargetPlatformVariant.all(),
    skip: kIsWeb, // [intended] we don't supply the cut/copy/paste buttons on the web.
  );

  testWidgets('spellCheckConfiguration passes through to EditableText', (
    WidgetTester tester,
  ) async {
    final SpellCheckConfiguration mySpellCheckConfiguration = SpellCheckConfiguration(
      spellCheckService: DefaultSpellCheckService(),
      misspelledTextStyle: TextField.materialMisspelledTextStyle,
    );

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: TextFormField(spellCheckConfiguration: mySpellCheckConfiguration)),
      ),
    );

    expect(find.byType(EditableText), findsOneWidget);

    final EditableText editableText = tester.widget(find.byType(EditableText));

    // Can't do equality comparison on spellCheckConfiguration itself because it
    // will have been copied.
    expect(
      editableText.spellCheckConfiguration?.spellCheckService,
      equals(mySpellCheckConfiguration.spellCheckService),
    );
    expect(
      editableText.spellCheckConfiguration?.misspelledTextStyle,
      equals(mySpellCheckConfiguration.misspelledTextStyle),
    );
  });

  testWidgets('magnifierConfiguration passes through to EditableText', (WidgetTester tester) async {
    final TextMagnifierConfiguration myTextMagnifierConfiguration = TextMagnifierConfiguration(
      magnifierBuilder: (
        BuildContext context,
        MagnifierController controller,
        ValueNotifier<MagnifierInfo> notifier,
      ) {
        return const Placeholder();
      },
    );

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: TextFormField(magnifierConfiguration: myTextMagnifierConfiguration)),
      ),
    );

    expect(find.byType(EditableText), findsOneWidget);

    final EditableText editableText = tester.widget(find.byType(EditableText));
    expect(editableText.magnifierConfiguration, equals(myTextMagnifierConfiguration));
  });

  testWidgets('Passes undoController to undoController TextField', (WidgetTester tester) async {
    final UndoHistoryController undoController = UndoHistoryController(
      value: UndoHistoryValue.empty,
    );
    addTearDown(undoController.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(child: Center(child: TextFormField(undoController: undoController))),
      ),
    );

    final Finder textFieldFinder = find.byType(TextField);
    expect(textFieldFinder, findsOneWidget);

    final TextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.undoController, undoController);
  });

  testWidgets('Passes cursorOpacityAnimates to cursorOpacityAnimates TextField', (
    WidgetTester tester,
  ) async {
    const bool cursorOpacityAnimates = true;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(child: TextFormField(cursorOpacityAnimates: cursorOpacityAnimates)),
        ),
      ),
    );

    final Finder textFieldFinder = find.byType(TextField);
    expect(textFieldFinder, findsOneWidget);

    final TextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.cursorOpacityAnimates, cursorOpacityAnimates);
  });

  testWidgets('Passes contentInsertionConfiguration to contentInsertionConfiguration TextField', (
    WidgetTester tester,
  ) async {
    final ContentInsertionConfiguration contentInsertionConfiguration =
        ContentInsertionConfiguration(onContentInserted: (KeyboardInsertedContent value) {});

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: TextFormField(contentInsertionConfiguration: contentInsertionConfiguration),
          ),
        ),
      ),
    );

    final Finder textFieldFinder = find.byType(TextField);
    expect(textFieldFinder, findsOneWidget);

    final TextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.contentInsertionConfiguration, contentInsertionConfiguration);
  });

  testWidgets('Passes clipBehavior to clipBehavior TextField', (WidgetTester tester) async {
    const Clip clipBehavior = Clip.antiAlias;

    await tester.pumpWidget(
      MaterialApp(home: Material(child: Center(child: TextFormField(clipBehavior: clipBehavior)))),
    );

    final Finder textFieldFinder = find.byType(TextField);
    expect(textFieldFinder, findsOneWidget);

    final TextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.clipBehavior, clipBehavior);
  });

  testWidgets('Passes scribbleEnabled to scribbleEnabled TextField', (WidgetTester tester) async {
    const bool scribbleEnabled = false;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(child: Center(child: TextFormField(scribbleEnabled: scribbleEnabled))),
      ),
    );

    final Finder textFieldFinder = find.byType(TextField);
    expect(textFieldFinder, findsOneWidget);

    final TextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.scribbleEnabled, scribbleEnabled);
  });

  testWidgets('Passes canRequestFocus to canRequestFocus TextField', (WidgetTester tester) async {
    const bool canRequestFocus = false;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(child: Center(child: TextFormField(canRequestFocus: canRequestFocus))),
      ),
    );

    final Finder textFieldFinder = find.byType(TextField);
    expect(textFieldFinder, findsOneWidget);

    final TextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.canRequestFocus, canRequestFocus);
  });

  testWidgets('Passes onAppPrivateCommand to onAppPrivateCommand TextField', (
    WidgetTester tester,
  ) async {
    void onAppPrivateCommand(String action, Map<String, dynamic> data) {}

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(child: TextFormField(onAppPrivateCommand: onAppPrivateCommand)),
        ),
      ),
    );

    final Finder textFieldFinder = find.byType(TextField);
    expect(textFieldFinder, findsOneWidget);

    final TextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.onAppPrivateCommand, onAppPrivateCommand);
  });

  testWidgets('Passes selectionHeightStyle to selectionHeightStyle TextField', (
    WidgetTester tester,
  ) async {
    const BoxHeightStyle selectionHeightStyle = BoxHeightStyle.max;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(child: TextFormField(selectionHeightStyle: selectionHeightStyle)),
        ),
      ),
    );

    final Finder textFieldFinder = find.byType(TextField);
    expect(textFieldFinder, findsOneWidget);

    final TextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.selectionHeightStyle, selectionHeightStyle);
  });

  testWidgets('Passes selectionWidthStyle to selectionWidthStyle TextField', (
    WidgetTester tester,
  ) async {
    const BoxWidthStyle selectionWidthStyle = BoxWidthStyle.max;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(child: TextFormField(selectionWidthStyle: selectionWidthStyle)),
        ),
      ),
    );

    final Finder textFieldFinder = find.byType(TextField);
    expect(textFieldFinder, findsOneWidget);

    final TextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.selectionWidthStyle, selectionWidthStyle);
  });

  testWidgets('Passes dragStartBehavior to dragStartBehavior TextField', (
    WidgetTester tester,
  ) async {
    const DragStartBehavior dragStartBehavior = DragStartBehavior.down;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(child: Center(child: TextFormField(dragStartBehavior: dragStartBehavior))),
      ),
    );

    final Finder textFieldFinder = find.byType(TextField);
    expect(textFieldFinder, findsOneWidget);

    final TextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.dragStartBehavior, dragStartBehavior);
  });

  testWidgets('Passes onTapAlwaysCalled to onTapAlwaysCalled TextField', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(home: Material(child: Center(child: TextFormField(onTapAlwaysCalled: true)))),
    );

    final Finder textFieldFinder = find.byType(TextField);
    expect(textFieldFinder, findsOneWidget);

    final TextField textFieldWidget = tester.widget(textFieldFinder);
    expect(textFieldWidget.onTapAlwaysCalled, isTrue);
  });

  testWidgets('Error color for cursor while validating', (WidgetTester tester) async {
    const Color themeErrorColor = Color(0xff111111);
    const Color errorStyleColor = Color(0xff777777);
    const Color cursorErrorColor = Color(0xffbbbbbb);

    Widget buildWidget({Color? errorStyleColor, Color? cursorErrorColor}) {
      return MaterialApp(
        theme: ThemeData(colorScheme: const ColorScheme.light(error: themeErrorColor)),
        home: Material(
          child: Center(
            child: TextFormField(
              enabled: true,
              autovalidateMode: AutovalidateMode.always,
              decoration: InputDecoration(errorStyle: TextStyle(color: errorStyleColor)),
              cursorErrorColor: cursorErrorColor,
              validator: (String? value) {
                return 'Please enter value';
              },
            ),
          ),
        ),
      );
    }

    Future<void> runTest(Widget widget, {required Color expectedColor}) async {
      await tester.pumpWidget(widget);
      await tester.enterText(find.byType(TextField), 'a');
      final EditableText textField = tester.widget(find.byType(EditableText).first);
      await tester.pump();
      expect(textField.cursorColor, expectedColor);
    }

    await runTest(buildWidget(), expectedColor: themeErrorColor);
    await runTest(buildWidget(errorStyleColor: errorStyleColor), expectedColor: errorStyleColor);
    await runTest(buildWidget(cursorErrorColor: cursorErrorColor), expectedColor: cursorErrorColor);
    await runTest(
      buildWidget(errorStyleColor: errorStyleColor, cursorErrorColor: cursorErrorColor),
      expectedColor: cursorErrorColor,
    );
  });

  testWidgets('TextFormField onChanged is called when the form is reset', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/123009.
    final GlobalKey<FormFieldState<String>> stateKey = GlobalKey<FormFieldState<String>>();
    final GlobalKey<FormState> formKey = GlobalKey<FormState>();
    String value = 'initialValue';

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Form(
            key: formKey,
            child: TextFormField(
              key: stateKey,
              initialValue: value,
              onChanged: (String newValue) {
                value = newValue;
              },
            ),
          ),
        ),
      ),
    );

    // Initial value is 'initialValue'.
    expect(stateKey.currentState!.value, 'initialValue');
    expect(value, 'initialValue');

    // Change value to 'changedValue'.
    await tester.enterText(find.byType(TextField), 'changedValue');
    expect(stateKey.currentState!.value, 'changedValue');
    expect(value, 'changedValue');

    // Should be back to 'initialValue' when the form is reset.
    formKey.currentState!.reset();
    await tester.pump();
    expect(stateKey.currentState!.value, 'initialValue');
    expect(value, 'initialValue');
  });

  testWidgets('isValid returns false when forceErrorText is set and will change error display', (
    WidgetTester tester,
  ) async {
    final GlobalKey<FormFieldState<String>> fieldKey1 = GlobalKey<FormFieldState<String>>();
    final GlobalKey<FormFieldState<String>> fieldKey2 = GlobalKey<FormFieldState<String>>();
    const String forceErrorText = 'Forcing error.';
    const String validString = 'Valid string';
    String? validator(String? s) => s == validString ? null : 'Error text';

    await tester.pumpWidget(
      MaterialApp(
        home: MediaQuery(
          data: const MediaQueryData(),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Center(
              child: Material(
                child: Form(
                  child: ListView(
                    children: <Widget>[
                      TextFormField(
                        key: fieldKey1,
                        initialValue: validString,
                        validator: validator,
                        autovalidateMode: AutovalidateMode.disabled,
                      ),
                      TextFormField(
                        key: fieldKey2,
                        initialValue: '',
                        forceErrorText: forceErrorText,
                        validator: validator,
                        autovalidateMode: AutovalidateMode.disabled,
                      ),
                    ],
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );

    expect(fieldKey1.currentState!.isValid, isTrue);
    expect(fieldKey1.currentState!.hasError, isFalse);
    expect(fieldKey2.currentState!.isValid, isFalse);
    expect(fieldKey2.currentState!.hasError, isTrue);
  });

  testWidgets('forceErrorText will override InputDecoration.error when both are provided', (
    WidgetTester tester,
  ) async {
    const String forceErrorText = 'Forcing error';
    const String decorationErrorText = 'Decoration';

    await tester.pumpWidget(
      MaterialApp(
        home: MediaQuery(
          data: const MediaQueryData(),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Center(
              child: Material(
                child: Form(
                  child: TextFormField(
                    forceErrorText: forceErrorText,
                    decoration: const InputDecoration(errorText: decorationErrorText),
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );

    expect(find.text(forceErrorText), findsOne);
    expect(find.text(decorationErrorText), findsNothing);
  });
}
