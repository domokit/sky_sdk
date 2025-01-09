// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/clipboard_utils.dart';
import '../widgets/editable_text_utils.dart'
    show findRenderEditable, globalize, textOffsetToPosition;

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();
  final MockClipboard mockClipboard = MockClipboard();

  setUp(() async {
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      mockClipboard.handleMethodCall,
    );
    // Fill the clipboard so that the Paste option is available in the text
    // selection menu.
    await Clipboard.setData(const ClipboardData(text: 'clipboard data'));
  });

  tearDown(() {
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      null,
    );
  });

  group('canSelectAll', () {
    Widget createEditableText({required Key key, String? text, TextSelection? selection}) {
      final TextEditingController controller = TextEditingController(text: text)
        ..selection = selection ?? const TextSelection.collapsed(offset: -1);
      addTearDown(controller.dispose);
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);

      return MaterialApp(
        home: EditableText(
          key: key,
          controller: controller,
          focusNode: focusNode,
          style: const TextStyle(),
          cursorColor: Colors.black,
          backgroundCursorColor: Colors.black,
        ),
      );
    }

    testWidgets('should return false when there is no text', (WidgetTester tester) async {
      final GlobalKey<EditableTextState> key = GlobalKey();
      await tester.pumpWidget(createEditableText(key: key));
      expect(materialTextSelectionControls.canSelectAll(key.currentState!), false);
    });

    testWidgets('should return true when there is text and collapsed selection', (
      WidgetTester tester,
    ) async {
      final GlobalKey<EditableTextState> key = GlobalKey();
      await tester.pumpWidget(createEditableText(key: key, text: '123'));
      expect(materialTextSelectionControls.canSelectAll(key.currentState!), true);
    });

    testWidgets('should return true when there is text and partial uncollapsed selection', (
      WidgetTester tester,
    ) async {
      final GlobalKey<EditableTextState> key = GlobalKey();
      await tester.pumpWidget(
        createEditableText(
          key: key,
          text: '123',
          selection: const TextSelection(baseOffset: 1, extentOffset: 2),
        ),
      );
      expect(materialTextSelectionControls.canSelectAll(key.currentState!), true);
    });

    testWidgets('should return false when there is text and full selection', (
      WidgetTester tester,
    ) async {
      final GlobalKey<EditableTextState> key = GlobalKey();
      await tester.pumpWidget(
        createEditableText(
          key: key,
          text: '123',
          selection: const TextSelection(baseOffset: 0, extentOffset: 3),
        ),
      );
      expect(materialTextSelectionControls.canSelectAll(key.currentState!), false);
    });
  });

  group('Text selection menu overflow (Android)', () {
    testWidgets(
      'All menu items show when they fit.',
      (WidgetTester tester) async {
        final TextEditingController controller = TextEditingController(text: 'abc def ghi');
        addTearDown(controller.dispose);
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(platform: TargetPlatform.android),
            home: Directionality(
              textDirection: TextDirection.ltr,
              child: MediaQuery(
                data: const MediaQueryData(size: Size(800.0, 600.0)),
                child: Center(child: Material(child: TextField(controller: controller))),
              ),
            ),
          ),
        );

        // Initially, the menu isn't shown at all.
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select all'), findsNothing);
        expect(find.byType(IconButton), findsNothing);

        // Tap to place the cursor in the field, then tap the handle to show the
        // selection menu.
        await tester.tap(find.byType(TextField));
        await tester.pumpAndSettle();
        final RenderEditable renderEditable = findRenderEditable(tester);
        final List<TextSelectionPoint> endpoints = globalize(
          renderEditable.getEndpointsForSelection(controller.selection),
          renderEditable,
        );
        expect(endpoints.length, 1);
        final Offset handlePos = endpoints[0].point + const Offset(0.0, 1.0);
        await tester.tapAt(handlePos, pointer: 7);
        await tester.pumpAndSettle();
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsOneWidget);
        expect(find.text('Select all'), findsOneWidget);
        expect(find.byType(IconButton), findsNothing);

        // Long press to select a word and show the full selection menu.
        final Offset textOffset = textOffsetToPosition(tester, 1);
        await tester.longPressAt(textOffset);
        await tester.pump();
        await tester.pump();

        // The full menu is shown without the more button.
        expect(find.text('Cut'), findsOneWidget);
        expect(find.text('Copy'), findsOneWidget);
        expect(find.text('Paste'), findsOneWidget);
        expect(find.text('Select all'), findsOneWidget);
        expect(find.byType(IconButton), findsNothing);
      },
      skip: isBrowser, // [intended] We do not use Flutter-rendered context menu on the Web.
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
    );

    testWidgets(
      "When menu items don't fit, an overflow menu is used.",
      (WidgetTester tester) async {
        // Set the screen size to more narrow, so that Select all can't fit.
        tester.view.physicalSize = const Size(1000, 800);
        addTearDown(tester.view.reset);

        final TextEditingController controller = TextEditingController(text: 'abc def ghi');
        addTearDown(controller.dispose);
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(platform: TargetPlatform.android),
            home: Directionality(
              textDirection: TextDirection.ltr,
              child: MediaQuery(
                data: const MediaQueryData(size: Size(800.0, 600.0)),
                child: Center(child: Material(child: TextField(controller: controller))),
              ),
            ),
          ),
        );

        // Initially, the menu isn't shown at all.
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select all'), findsNothing);
        expect(find.byType(IconButton), findsNothing);

        // Long press to show the menu.
        final Offset textOffset = textOffsetToPosition(tester, 1);
        await tester.longPressAt(textOffset);
        await tester.pumpAndSettle();

        // The last button is missing, and a more button is shown.
        expect(find.text('Cut'), findsOneWidget);
        expect(find.text('Copy'), findsOneWidget);
        expect(find.text('Paste'), findsOneWidget);
        expect(find.text('Select all'), findsNothing);
        expect(find.byType(IconButton), findsOneWidget);
        final Offset cutOffset = tester.getTopLeft(find.text('Cut'));

        // Tapping the button shows the overflow menu.
        await tester.tap(find.byType(IconButton));
        await tester.pumpAndSettle();
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select all'), findsOneWidget);
        expect(find.byType(IconButton), findsOneWidget);

        // The back button is at the bottom of the overflow menu.
        final Offset selectAllOffset = tester.getTopLeft(find.text('Select all'));
        final Offset moreOffset = tester.getTopLeft(find.byType(IconButton));
        expect(moreOffset.dy, greaterThan(selectAllOffset.dy));

        // The overflow menu grows upward.
        expect(selectAllOffset.dy, lessThan(cutOffset.dy));

        // Tapping the back button shows the selection menu again.
        expect(find.byType(IconButton), findsOneWidget);
        await tester.tap(find.byType(IconButton));
        await tester.pumpAndSettle();
        expect(find.text('Cut'), findsOneWidget);
        expect(find.text('Copy'), findsOneWidget);
        expect(find.text('Paste'), findsOneWidget);
        expect(find.text('Select all'), findsNothing);
        expect(find.byType(IconButton), findsOneWidget);
      },
      skip: isBrowser, // [intended] We do not use Flutter-rendered context menu on the Web.
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
    );

    testWidgets(
      'A smaller menu bumps more items to the overflow menu.',
      (WidgetTester tester) async {
        // Set the screen size so narrow that only Cut and Copy can fit.
        tester.view.physicalSize = const Size(800, 800);
        addTearDown(tester.view.reset);

        final TextEditingController controller = TextEditingController(text: 'abc def ghi');
        addTearDown(controller.dispose);
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(platform: TargetPlatform.android),
            home: Directionality(
              textDirection: TextDirection.ltr,
              child: MediaQuery(
                data: const MediaQueryData(size: Size(800.0, 600.0)),
                child: Center(child: Material(child: TextField(controller: controller))),
              ),
            ),
          ),
        );

        // Initially, the menu isn't shown at all.
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select all'), findsNothing);
        expect(find.byType(IconButton), findsNothing);

        // Long press to show the menu.
        final Offset textOffset = textOffsetToPosition(tester, 1);
        await tester.longPressAt(textOffset);
        await tester.pumpAndSettle();

        // The last two buttons are missing, and a more button is shown.
        expect(find.text('Cut'), findsOneWidget);
        expect(find.text('Copy'), findsOneWidget);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select all'), findsNothing);
        expect(find.byType(IconButton), findsOneWidget);

        // Tapping the button shows the overflow menu, which contains both buttons
        // missing from the main menu, and a back button.
        await tester.tap(find.byType(IconButton));
        await tester.pumpAndSettle();
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsOneWidget);
        expect(find.text('Select all'), findsOneWidget);
        expect(find.byType(IconButton), findsOneWidget);

        // Tapping the back button shows the selection menu again.
        await tester.tap(find.byType(IconButton));
        await tester.pumpAndSettle();
        expect(find.text('Cut'), findsOneWidget);
        expect(find.text('Copy'), findsOneWidget);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select all'), findsNothing);
        expect(find.byType(IconButton), findsOneWidget);
      },
      skip: isBrowser, // [intended] We do not use Flutter-rendered context menu on the Web.
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
    );

    testWidgets(
      'When the menu renders below the text, the overflow menu back button is at the top.',
      (WidgetTester tester) async {
        // Set the screen size to more narrow, so that Select all can't fit.
        tester.view.physicalSize = const Size(1000, 800);
        addTearDown(tester.view.reset);

        final TextEditingController controller = TextEditingController(text: 'abc def ghi');
        addTearDown(controller.dispose);
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(platform: TargetPlatform.android),
            home: Directionality(
              textDirection: TextDirection.ltr,
              child: MediaQuery(
                data: const MediaQueryData(size: Size(800.0, 600.0)),
                child: Align(
                  alignment: Alignment.topLeft,
                  child: Material(child: TextField(controller: controller)),
                ),
              ),
            ),
          ),
        );

        // Initially, the menu isn't shown at all.
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select all'), findsNothing);
        expect(find.byType(IconButton), findsNothing);

        // Long press to show the menu.
        final Offset textOffset = textOffsetToPosition(tester, 1);
        await tester.longPressAt(textOffset);
        await tester.pumpAndSettle();

        // The last button is missing, and a more button is shown.
        expect(find.text('Cut'), findsOneWidget);
        expect(find.text('Copy'), findsOneWidget);
        expect(find.text('Paste'), findsOneWidget);
        expect(find.text('Select all'), findsNothing);
        expect(find.byType(IconButton), findsOneWidget);
        final Offset cutOffset = tester.getTopLeft(find.text('Cut'));

        // Tapping the button shows the overflow menu.
        await tester.tap(find.byType(IconButton));
        await tester.pumpAndSettle();
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select all'), findsOneWidget);
        expect(find.byType(IconButton), findsOneWidget);

        // The back button is at the top of the overflow menu.
        final Offset selectAllOffset = tester.getTopLeft(find.text('Select all'));
        final Offset moreOffset = tester.getTopLeft(find.byType(IconButton));
        expect(moreOffset.dy, lessThan(selectAllOffset.dy));

        // The overflow menu grows downward.
        expect(selectAllOffset.dy, greaterThan(cutOffset.dy));

        // Tapping the back button shows the selection menu again.
        await tester.tap(find.byType(IconButton));
        await tester.pumpAndSettle();
        expect(find.text('Cut'), findsOneWidget);
        expect(find.text('Copy'), findsOneWidget);
        expect(find.text('Paste'), findsOneWidget);
        expect(find.text('Select all'), findsNothing);
        expect(find.byType(IconButton), findsOneWidget);
      },
      skip: isBrowser, // [intended] We do not use Flutter-rendered context menu on the Web.
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
    );

    testWidgets(
      'When the menu items change, the menu is closed and _closedWidth reset.',
      (WidgetTester tester) async {
        // Set the screen size to more narrow, so that Select all can't fit.
        tester.view.physicalSize = const Size(1000, 800);
        addTearDown(tester.view.reset);

        final TextEditingController controller = TextEditingController(text: 'abc def ghi');
        addTearDown(controller.dispose);
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(platform: TargetPlatform.android, useMaterial3: false),
            home: Directionality(
              textDirection: TextDirection.ltr,
              child: MediaQuery(
                data: const MediaQueryData(size: Size(800.0, 600.0)),
                child: Align(
                  alignment: Alignment.topLeft,
                  child: Material(child: TextField(controller: controller)),
                ),
              ),
            ),
          ),
        );

        // Initially, the menu isn't shown at all.
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Share'), findsNothing);
        expect(find.text('Select all'), findsNothing);
        expect(find.byType(IconButton), findsNothing); // 'More' button.

        // Tap to place the cursor and tap again to show the menu without a
        // selection.
        await tester.tapAt(textOffsetToPosition(tester, 0));
        await tester.pumpAndSettle();
        final RenderEditable renderEditable = findRenderEditable(tester);
        final List<TextSelectionPoint> endpoints = globalize(
          renderEditable.getEndpointsForSelection(controller.selection),
          renderEditable,
        );
        expect(endpoints.length, 1);
        final Offset handlePos = endpoints[0].point + const Offset(0.0, 1.0);
        await tester.tapAt(handlePos, pointer: 7);
        await tester.pumpAndSettle();
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsOneWidget);
        expect(find.text('Share'), findsNothing);
        expect(find.text('Select all'), findsOneWidget);
        expect(find.byType(IconButton), findsNothing);

        // Tap Select all and measure the usual position of Cut, without
        // _closedWidth having been used yet.
        await tester.tap(find.text('Select all'));
        await tester.pumpAndSettle();
        expect(find.text('Cut'), findsOneWidget);
        expect(find.text('Copy'), findsOneWidget);
        expect(find.text('Paste'), findsOneWidget);
        expect(find.text('Share'), findsNothing);
        expect(find.text('Select all'), findsNothing);
        expect(find.byType(IconButton), findsOneWidget); // 'More' button.
        final Offset cutOffset = tester.getTopLeft(find.text('Cut'));

        // Tap to clear the selection.
        await tester.tapAt(textOffsetToPosition(tester, 0));
        await tester.pumpAndSettle();
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select all'), findsNothing);
        expect(find.byType(IconButton), findsNothing); // 'More' button.

        // Long press to show the menu.
        await tester.longPressAt(textOffsetToPosition(tester, 1));
        await tester.pumpAndSettle();

        // The last buttons (share and select all) are missing, and a more button is shown.
        expect(find.text('Cut'), findsOneWidget);
        expect(find.text('Copy'), findsOneWidget);
        expect(find.text('Paste'), findsOneWidget);
        expect(find.text('Share'), findsNothing);
        expect(find.text('Select all'), findsNothing);
        expect(find.byType(IconButton), findsOneWidget); // 'More' button.

        // Tapping the more button shows the overflow menu.
        await tester.tap(find.byType(IconButton));
        await tester.pumpAndSettle();
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Share'), findsOneWidget);
        expect(find.text('Select all'), findsOneWidget);
        expect(find.byType(IconButton), findsOneWidget); // Back button.

        // Tapping 'Select all' closes the overflow menu.
        await tester.tap(find.text('Select all'));
        await tester.pumpAndSettle();
        expect(find.text('Cut'), findsOneWidget);
        expect(find.text('Copy'), findsOneWidget);
        expect(find.text('Paste'), findsOneWidget);
        expect(find.text('Share'), findsNothing);
        expect(find.text('Select all'), findsNothing);
        expect(find.byType(IconButton), findsOneWidget); // 'More' button.
        final Offset newCutOffset = tester.getTopLeft(find.text('Cut'));
        expect(newCutOffset, equals(cutOffset));
      },
      skip: isBrowser, // [intended] We do not use Flutter-rendered context menu on the Web.
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
    );
  });

  group('menu position', () {
    testWidgets(
      'When renders below a block of text, menu appears below bottom endpoint',
      (WidgetTester tester) async {
        final TextEditingController controller = TextEditingController(
          text: 'abc\ndef\nghi\njkl\nmno\npqr',
        );
        addTearDown(controller.dispose);
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(platform: TargetPlatform.android),
            home: Directionality(
              textDirection: TextDirection.ltr,
              child: MediaQuery(
                data: const MediaQueryData(size: Size(800.0, 600.0)),
                child: Align(
                  alignment: Alignment.topLeft,
                  child: Material(child: TextField(controller: controller)),
                ),
              ),
            ),
          ),
        );

        // Initially, the menu isn't shown at all.
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select all'), findsNothing);
        expect(find.byType(IconButton), findsNothing);

        // Tap to place the cursor in the field, then tap the handle to show the
        // selection menu.
        await tester.tap(find.byType(TextField));
        await tester.pumpAndSettle();
        RenderEditable renderEditable = findRenderEditable(tester);
        List<TextSelectionPoint> endpoints = globalize(
          renderEditable.getEndpointsForSelection(controller.selection),
          renderEditable,
        );
        expect(endpoints.length, 1);
        final Offset handlePos = endpoints[0].point + const Offset(0.0, 1.0);
        await tester.tapAt(handlePos, pointer: 7);
        await tester.pumpAndSettle();
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsOneWidget);
        expect(find.text('Select all'), findsOneWidget);
        expect(find.byType(IconButton), findsNothing);

        // Tap to select all.
        await tester.tap(find.text('Select all'));
        await tester.pumpAndSettle();

        // Only Cut, Copy, and Paste are shown.
        expect(find.text('Cut'), findsOneWidget);
        expect(find.text('Copy'), findsOneWidget);
        expect(find.text('Paste'), findsOneWidget);
        expect(find.text('Select all'), findsNothing);
        expect(find.byType(IconButton), findsNothing);

        // The menu appears below the bottom handle.
        renderEditable = findRenderEditable(tester);
        endpoints = globalize(
          renderEditable.getEndpointsForSelection(controller.selection),
          renderEditable,
        );
        expect(endpoints.length, 2);
        final Offset bottomHandlePos = endpoints[1].point;
        final Offset cutOffset = tester.getTopLeft(find.text('Cut'));
        expect(cutOffset.dy, greaterThan(bottomHandlePos.dy));
      },
      skip: isBrowser, // [intended] We do not use Flutter-rendered context menu on the Web.
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
    );

    testWidgets(
      'When selecting multiple lines over max lines',
      (WidgetTester tester) async {
        final TextEditingController controller = TextEditingController(
          text: 'abc\ndef\nghi\njkl\nmno\npqr',
        );
        addTearDown(controller.dispose);
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(platform: TargetPlatform.android),
            home: Directionality(
              textDirection: TextDirection.ltr,
              child: MediaQuery(
                data: const MediaQueryData(size: Size(800.0, 600.0)),
                child: Align(
                  alignment: Alignment.bottomCenter,
                  child: Material(
                    child: TextField(
                      decoration: const InputDecoration(contentPadding: EdgeInsets.all(8.0)),
                      style: const TextStyle(fontSize: 32, height: 1),
                      maxLines: 2,
                      controller: controller,
                    ),
                  ),
                ),
              ),
            ),
          ),
        );

        // Initially, the menu isn't shown at all.
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select all'), findsNothing);
        expect(find.byType(IconButton), findsNothing);

        // Tap to place the cursor in the field, then tap the handle to show the
        // selection menu.
        await tester.tap(find.byType(TextField));
        await tester.pumpAndSettle();
        final RenderEditable renderEditable = findRenderEditable(tester);
        final List<TextSelectionPoint> endpoints = globalize(
          renderEditable.getEndpointsForSelection(controller.selection),
          renderEditable,
        );
        expect(endpoints.length, 1);
        final Offset handlePos = endpoints[0].point + const Offset(0.0, 1.0);
        await tester.tapAt(handlePos, pointer: 7);
        await tester.pumpAndSettle();
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsOneWidget);
        expect(find.text('Select all'), findsOneWidget);
        expect(find.byType(IconButton), findsNothing);

        // Tap to select all.
        await tester.tap(find.text('Select all'));
        await tester.pumpAndSettle();

        // Only Cut, Copy, and Paste are shown.
        expect(find.text('Cut'), findsOneWidget);
        expect(find.text('Copy'), findsOneWidget);
        expect(find.text('Paste'), findsOneWidget);
        expect(find.text('Select all'), findsNothing);
        expect(find.byType(IconButton), findsNothing);

        // The menu appears at the top of the visible selection.
        final Offset selectionOffset = tester.getTopLeft(
          find.byType(TextSelectionToolbarTextButton).first,
        );
        final Offset textFieldOffset = tester.getTopLeft(find.byType(TextField));

        // 44.0 + 8.0 - 8.0 = _kToolbarHeight + _kToolbarContentDistance - contentPadding
        expect(selectionOffset.dy + 44.0 + 8.0 - 8.0, equals(textFieldOffset.dy));
      },
      skip: isBrowser, // [intended] the selection menu isn't required by web
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
    );
  });

  group('material handles', () {
    testWidgets('draws transparent handle correctly', (WidgetTester tester) async {
      await tester.pumpWidget(
        RepaintBoundary(
          child: Theme(
            data: ThemeData(
              textSelectionTheme: const TextSelectionThemeData(
                selectionHandleColor: Color(0x550000AA),
              ),
            ),
            child: Builder(
              builder: (BuildContext context) {
                return Container(
                  color: Colors.white,
                  height: 800,
                  width: 800,
                  child: Padding(
                    padding: const EdgeInsets.symmetric(horizontal: 250),
                    child: FittedBox(
                      child: materialTextSelectionControls.buildHandle(
                        context,
                        TextSelectionHandleType.right,
                        10.0,
                      ),
                    ),
                  ),
                );
              },
            ),
          ),
        ),
      );

      await expectLater(find.byType(RepaintBoundary), matchesGoldenFile('transparent_handle.png'));
    });

    testWidgets('works with 3 positional parameters', (WidgetTester tester) async {
      await tester.pumpWidget(
        Theme(
          data: ThemeData(
            textSelectionTheme: const TextSelectionThemeData(
              selectionHandleColor: Color(0x550000AA),
            ),
          ),
          child: Builder(
            builder: (BuildContext context) {
              return Container(
                color: Colors.white,
                height: 800,
                width: 800,
                child: Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 250),
                  child: FittedBox(
                    child: materialTextSelectionControls.buildHandle(
                      context,
                      TextSelectionHandleType.right,
                      10.0,
                    ),
                  ),
                ),
              );
            },
          ),
        ),
      );

      // No expect here as this should simply compile / not throw any
      // exceptions while building. The test will fail if this either does
      // not compile or if the tester catches an exception, which we do
      // not take here.
    });
  });

  testWidgets(
    'Paste only appears when clipboard has contents',
    (WidgetTester tester) async {
      final TextEditingController controller = TextEditingController(
        text: 'Atwater Peel Sherbrooke Bonaventure',
      );
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Material(child: Column(children: <Widget>[TextField(controller: controller)])),
        ),
      );

      // Make sure the clipboard is empty to start.
      await Clipboard.setData(const ClipboardData(text: ''));

      // Double tap to select the first word.
      const int index = 4;
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pumpAndSettle();

      // No Paste yet, because nothing has been copied.
      expect(find.text('Paste'), findsNothing);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Select all'), findsOneWidget);

      // Tap copy to add something to the clipboard and close the menu.
      await tester.tapAt(tester.getCenter(find.text('Copy')));
      await tester.pumpAndSettle();
      expect(find.text('Copy'), findsNothing);
      expect(find.text('Cut'), findsNothing);
      expect(find.text('Select all'), findsNothing);

      // Double tap to show the menu again.
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pumpAndSettle();

      // Paste now shows.
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);
      expect(find.text('Select all'), findsOneWidget);
    },
    skip: isBrowser, // [intended] we don't supply the cut/copy/paste buttons on the web.
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.android}),
  );
}
