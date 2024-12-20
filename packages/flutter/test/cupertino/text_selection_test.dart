// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'dart:ui' as ui show BoxHeightStyle;

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/clipboard_utils.dart';
import '../widgets/editable_text_utils.dart' show findRenderEditable, textOffsetToPosition;

class _LongCupertinoLocalizationsDelegate extends LocalizationsDelegate<CupertinoLocalizations> {
  const _LongCupertinoLocalizationsDelegate();

  @override
  bool isSupported(Locale locale) => locale.languageCode == 'en';

  @override
  Future<_LongCupertinoLocalizations> load(Locale locale) =>
      _LongCupertinoLocalizations.load(locale);

  @override
  bool shouldReload(_LongCupertinoLocalizationsDelegate old) => false;

  @override
  String toString() => '_LongCupertinoLocalizations.delegate(en_US)';
}

class _LongCupertinoLocalizations extends DefaultCupertinoLocalizations {
  const _LongCupertinoLocalizations();

  @override
  String get cutButtonLabel => 'Cutttttttttttttttttttttttttttttttttttttttttttt';
  @override
  String get copyButtonLabel => 'Copyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy';
  @override
  String get pasteButtonLabel => 'Pasteeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee';
  @override
  String get selectAllButtonLabel => 'Select Allllllllllllllllllllllllllllllll';

  static Future<_LongCupertinoLocalizations> load(Locale locale) {
    return SynchronousFuture<_LongCupertinoLocalizations>(const _LongCupertinoLocalizations());
  }

  static const LocalizationsDelegate<CupertinoLocalizations> delegate =
      _LongCupertinoLocalizationsDelegate();
}

const _LongCupertinoLocalizations _longLocalizations = _LongCupertinoLocalizations();

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();
  final MockClipboard mockClipboard = MockClipboard();

  List<TextSelectionPoint> globalize(Iterable<TextSelectionPoint> points, RenderBox box) {
    return points.map<TextSelectionPoint>((TextSelectionPoint point) {
      return TextSelectionPoint(box.localToGlobal(point.point), point.direction);
    }).toList();
  }

  setUp(() async {
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

  group('canSelectAll', () {
    Widget createEditableText({Key? key, String? text, TextSelection? selection}) {
      final TextEditingController controller = TextEditingController(text: text)
        ..selection = selection ?? const TextSelection.collapsed(offset: -1);
      addTearDown(controller.dispose);
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);

      return CupertinoApp(
        home: EditableText(
          key: key,
          controller: controller,
          focusNode: focusNode,
          style: const TextStyle(),
          cursorColor: const Color.fromARGB(0, 0, 0, 0),
          backgroundCursorColor: const Color.fromARGB(0, 0, 0, 0),
        ),
      );
    }

    testWidgets('should return false when there is no text', (WidgetTester tester) async {
      final GlobalKey<EditableTextState> key = GlobalKey();
      await tester.pumpWidget(createEditableText(key: key));
      expect(cupertinoTextSelectionControls.canSelectAll(key.currentState!), false);
    });

    testWidgets('should return true when there is text and collapsed selection', (
      WidgetTester tester,
    ) async {
      final GlobalKey<EditableTextState> key = GlobalKey();
      await tester.pumpWidget(createEditableText(key: key, text: '123'));
      expect(cupertinoTextSelectionControls.canSelectAll(key.currentState!), true);
    });

    testWidgets('should return false when there is text and partial uncollapsed selection', (
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
      expect(cupertinoTextSelectionControls.canSelectAll(key.currentState!), false);
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
      expect(cupertinoTextSelectionControls.canSelectAll(key.currentState!), false);
    });
  });

  group('cupertino handles', () {
    testWidgets('draws transparent handle correctly', (WidgetTester tester) async {
      await tester.pumpWidget(
        RepaintBoundary(
          child: CupertinoTheme(
            data: const CupertinoThemeData(primaryColor: Color(0x550000AA)),
            child: Builder(
              builder: (BuildContext context) {
                return Container(
                  color: CupertinoColors.white,
                  height: 800,
                  width: 800,
                  child: Padding(
                    padding: const EdgeInsets.symmetric(horizontal: 250),
                    child: FittedBox(
                      child: cupertinoTextSelectionControls.buildHandle(
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

      await expectLater(
        find.byType(RepaintBoundary),
        matchesGoldenFile('text_selection.handle.transparent.png'),
      );
    });
  });

  group('Text selection menu overflow (iOS)', () {
    Finder findOverflowNextButton() {
      return find.byWidgetPredicate(
        (Widget widget) =>
            widget is CustomPaint &&
            '${widget.painter?.runtimeType}' == '_RightCupertinoChevronPainter',
      );
    }

    Finder findOverflowBackButton() => find.byWidgetPredicate(
      (Widget widget) =>
          widget is CustomPaint &&
          '${widget.painter?.runtimeType}' == '_LeftCupertinoChevronPainter',
    );

    testWidgets(
      'All menu items show when they fit.',
      (WidgetTester tester) async {
        final TextEditingController controller = TextEditingController(text: 'abc def ghi');
        addTearDown(controller.dispose);
        await tester.pumpWidget(
          CupertinoApp(
            home: Directionality(
              textDirection: TextDirection.ltr,
              child: MediaQuery(
                data: const MediaQueryData(size: Size(800.0, 600.0)),
                child: Center(child: CupertinoTextField(autofocus: true, controller: controller)),
              ),
            ),
          ),
        );

        // This extra pump is so autofocus can propagate to renderEditable.
        await tester.pump();

        // Initially, the menu isn't shown at all.
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select All'), findsNothing);
        expect(findOverflowBackButton(), findsNothing);
        expect(findOverflowNextButton(), findsNothing);

        // Long press on an empty space to show the selection menu.
        await tester.longPressAt(textOffsetToPosition(tester, 4));
        await tester.pumpAndSettle();
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsOneWidget);
        expect(find.text('Select All'), findsOneWidget);
        expect(findOverflowBackButton(), findsNothing);
        expect(findOverflowNextButton(), findsNothing);

        // Double tap to select a word and show the full selection menu.
        final Offset textOffset = textOffsetToPosition(tester, 1);
        await tester.tapAt(textOffset);
        await tester.pump(const Duration(milliseconds: 200));
        await tester.tapAt(textOffset);
        await tester.pumpAndSettle();

        // The full menu is shown without the navigation buttons.
        expect(find.text('Cut'), findsOneWidget);
        expect(find.text('Copy'), findsOneWidget);
        expect(find.text('Paste'), findsOneWidget);
        expect(find.text('Select All'), findsNothing);
        expect(findOverflowBackButton(), findsNothing);
        expect(findOverflowNextButton(), findsNothing);
      },
      skip: isBrowser, // [intended] We do not use Flutter-rendered context menu on the Web.
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
    );

    testWidgets(
      "When a menu item doesn't fit, a second page is used.",
      (WidgetTester tester) async {
        // Set the screen size to more narrow, so that Paste can't fit.
        tester.view.physicalSize = const Size(1000, 800);
        addTearDown(tester.view.reset);

        final TextEditingController controller = TextEditingController(text: 'abc def ghi');
        addTearDown(controller.dispose);
        await tester.pumpWidget(
          CupertinoApp(
            home: Directionality(
              textDirection: TextDirection.ltr,
              child: MediaQuery(
                data: const MediaQueryData(size: Size(800.0, 600.0)),
                child: Center(child: CupertinoTextField(controller: controller)),
              ),
            ),
          ),
        );

        Future<void> tapNextButton() async {
          await tester.tapAt(tester.getCenter(findOverflowNextButton()));
          await tester.pumpAndSettle();
        }

        Future<void> tapBackButton() async {
          await tester.tapAt(tester.getCenter(findOverflowBackButton()));
          await tester.pumpAndSettle();
        }

        // Initially, the menu isn't shown at all.
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select All'), findsNothing);
        expect(find.text('Look Up'), findsNothing);
        expect(find.text('Search Web'), findsNothing);
        expect(findOverflowBackButton(), findsNothing);
        expect(findOverflowNextButton(), findsNothing);

        // Double tap to select a word and show the selection menu.
        final Offset textOffset = textOffsetToPosition(tester, 1);
        await tester.tapAt(textOffset);
        await tester.pump(const Duration(milliseconds: 200));
        await tester.tapAt(textOffset);
        await tester.pumpAndSettle();

        // The last button is missing, and a next button is shown.
        expect(find.text('Cut'), findsOneWidget);
        expect(find.text('Copy'), findsOneWidget);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select All'), findsNothing);
        expect(find.text('Look Up'), findsNothing);
        expect(find.text('Search Web'), findsNothing);
        expect(findOverflowBackButton(), findsNothing);
        expect(findOverflowNextButton(), findsOneWidget);

        // Tapping the next button shows both the overflow, back, and next buttons.
        await tapNextButton();
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsOneWidget);
        expect(find.text('Select All'), findsNothing);
        expect(find.text('Look Up'), findsNothing);
        expect(find.text('Search Web'), findsNothing);
        expect(findOverflowBackButton(), findsOneWidget);
        expect(findOverflowNextButton(), findsOneWidget);

        // Tapping the next button shows the next, back, and Look Up button
        await tapNextButton();
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select All'), findsNothing);
        expect(find.text('Look Up'), findsOneWidget);
        expect(find.text('Search Web'), findsNothing);
        expect(findOverflowBackButton(), findsOneWidget);
        expect(findOverflowNextButton(), findsOneWidget);

        // Tapping the next button shows the back and Search Web button
        await tapNextButton();
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select All'), findsNothing);
        expect(find.text('Look Up'), findsNothing);
        expect(find.text('Search Web'), findsOneWidget);
        expect(findOverflowBackButton(), findsOneWidget);
        expect(findOverflowNextButton(), findsOneWidget);

        // Tapping the back button thrice shows the first page again with the next button.
        await tapBackButton();
        await tapBackButton();
        await tapBackButton();
        expect(find.text('Cut'), findsOneWidget);
        expect(find.text('Copy'), findsOneWidget);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select All'), findsNothing);
        expect(find.text('Look Up'), findsNothing);
        expect(find.text('Search Web'), findsNothing);
        expect(findOverflowBackButton(), findsNothing);
        expect(findOverflowNextButton(), findsOneWidget);
      },
      skip: isBrowser, // [intended] We do not use Flutter-rendered context menu on the Web.
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
    );

    testWidgets(
      'A smaller menu puts each button on its own page.',
      (WidgetTester tester) async {
        // Set the screen size to more narrow, so that two buttons can't fit on
        // the same page.
        tester.view.physicalSize = const Size(640, 800);
        addTearDown(tester.view.reset);

        final TextEditingController controller = TextEditingController(text: 'abc def ghi');
        addTearDown(controller.dispose);
        await tester.pumpWidget(
          CupertinoApp(
            home: Directionality(
              textDirection: TextDirection.ltr,
              child: MediaQuery(
                data: const MediaQueryData(size: Size(800.0, 600.0)),
                child: Center(child: CupertinoTextField(controller: controller)),
              ),
            ),
          ),
        );

        Future<void> tapNextButton() async {
          await tester.tapAt(tester.getCenter(findOverflowNextButton()));
          await tester.pumpAndSettle();
        }

        Future<void> tapBackButton() async {
          await tester.tapAt(tester.getCenter(findOverflowBackButton()));
          await tester.pumpAndSettle();
        }

        // Initially, the menu isn't shown at all.
        expect(find.byType(CupertinoTextSelectionToolbarButton), findsNothing);
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select All'), findsNothing);
        expect(find.text('Look Up'), findsNothing);
        expect(find.text('Search Web'), findsNothing);
        expect(find.text('Share...'), findsNothing);
        expect(findOverflowBackButton(), findsNothing);
        expect(findOverflowNextButton(), findsNothing);

        // Double tap to select a word and show the selection menu.
        final Offset textOffset = textOffsetToPosition(tester, 1);
        await tester.tapAt(textOffset);
        await tester.pump(const Duration(milliseconds: 200));
        await tester.tapAt(textOffset);
        await tester.pumpAndSettle();

        // Only the first button fits, and a next button is shown.
        expect(find.byType(CupertinoTextSelectionToolbarButton), findsNWidgets(2));
        expect(find.text('Cut'), findsOneWidget);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select All'), findsNothing);
        expect(find.text('Look Up'), findsNothing);
        expect(find.text('Search Web'), findsNothing);
        expect(find.text('Share...'), findsNothing);
        expect(findOverflowBackButton(), findsNothing);
        expect(findOverflowNextButton(), findsOneWidget);

        // Tapping the next button shows Copy.
        await tapNextButton();
        expect(find.byType(CupertinoTextSelectionToolbarButton), findsNWidgets(3));
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsOneWidget);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select All'), findsNothing);
        expect(find.text('Look Up'), findsNothing);
        expect(find.text('Search Web'), findsNothing);
        expect(find.text('Share...'), findsNothing);
        expect(findOverflowBackButton(), findsOneWidget);
        expect(findOverflowNextButton(), findsOneWidget);

        // Tapping the next button again shows Paste
        await tapNextButton();
        expect(find.byType(CupertinoTextSelectionToolbarButton), findsNWidgets(3));
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsOneWidget);
        expect(find.text('Select All'), findsNothing);
        expect(find.text('Look Up'), findsNothing);
        expect(find.text('Search Web'), findsNothing);
        expect(find.text('Share...'), findsNothing);
        expect(findOverflowBackButton(), findsOneWidget);
        expect(findOverflowNextButton(), findsOneWidget);

        // Tapping the next button again shows the Look Up Button.
        await tapNextButton();
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select All'), findsNothing);
        expect(find.text('Look Up'), findsOneWidget);
        expect(find.text('Search Web'), findsNothing);
        expect(find.text('Share...'), findsNothing);
        expect(findOverflowBackButton(), findsOneWidget);
        expect(findOverflowNextButton(), findsOneWidget);

        // Tapping the next button again shows the Search Web Button.
        await tapNextButton();
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select All'), findsNothing);
        expect(find.text('Look Up'), findsNothing);
        expect(find.text('Search Web'), findsOneWidget);
        expect(find.text('Share...'), findsNothing);
        expect(findOverflowBackButton(), findsOneWidget);
        expect(findOverflowNextButton(), findsOneWidget);

        // Tapping the next button again shows the last page and the Share button
        await tapNextButton();
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select All'), findsNothing);
        expect(find.text('Look Up'), findsNothing);
        expect(find.text('Search Web'), findsNothing);
        expect(find.text('Share...'), findsOneWidget);
        expect(findOverflowBackButton(), findsOneWidget);
        expect(findOverflowNextButton(), findsNothing);

        // Tapping the back button 5 times shows the first page again.
        await tapBackButton();
        await tapBackButton();
        await tapBackButton();
        await tapBackButton();
        await tapBackButton();
        expect(find.byType(CupertinoTextSelectionToolbarButton), findsNWidgets(2));
        expect(find.text('Cut'), findsOneWidget);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select All'), findsNothing);
        expect(find.text('Look Up'), findsNothing);
        expect(find.text('Search Web'), findsNothing);
        expect(find.text('Share...'), findsNothing);
        expect(findOverflowBackButton(), findsNothing);
        expect(findOverflowNextButton(), findsOneWidget);
      },
      skip: isBrowser, // [intended] We do not use Flutter-rendered context menu on the Web.
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
    );

    testWidgets(
      'Handles very long locale strings',
      (WidgetTester tester) async {
        final TextEditingController controller = TextEditingController(text: 'abc def ghi');
        addTearDown(controller.dispose);
        await tester.pumpWidget(
          CupertinoApp(
            locale: const Locale('en', 'us'),
            localizationsDelegates: const <LocalizationsDelegate<dynamic>>[
              _LongCupertinoLocalizations.delegate,
              DefaultWidgetsLocalizations.delegate,
              DefaultMaterialLocalizations.delegate,
            ],
            home: Directionality(
              textDirection: TextDirection.ltr,
              child: MediaQuery(
                data: const MediaQueryData(size: Size(800.0, 600.0)),
                child: Center(child: CupertinoTextField(autofocus: true, controller: controller)),
              ),
            ),
          ),
        );

        // This extra pump is so autofocus can propagate to renderEditable.
        await tester.pump();

        // Initially, the menu isn't shown at all.
        expect(find.text(_longLocalizations.cutButtonLabel), findsNothing);
        expect(find.text(_longLocalizations.copyButtonLabel), findsNothing);
        expect(find.text(_longLocalizations.pasteButtonLabel), findsNothing);
        expect(find.text(_longLocalizations.selectAllButtonLabel), findsNothing);
        expect(findOverflowBackButton(), findsNothing);
        expect(findOverflowNextButton(), findsNothing);

        // Long press on an empty space to show the selection menu, with only the
        // paste button visible.
        await tester.longPressAt(textOffsetToPosition(tester, 4));
        await tester.pumpAndSettle();
        expect(find.text(_longLocalizations.cutButtonLabel), findsNothing);
        expect(find.text(_longLocalizations.copyButtonLabel), findsNothing);
        expect(find.text(_longLocalizations.pasteButtonLabel), findsOneWidget);
        expect(find.text(_longLocalizations.selectAllButtonLabel), findsNothing);
        expect(findOverflowBackButton(), findsNothing);
        expect(findOverflowNextButton(), findsOneWidget);

        // Tap next to go to the second and final page.
        await tester.tapAt(tester.getCenter(findOverflowNextButton()));
        await tester.pumpAndSettle();
        expect(find.text(_longLocalizations.cutButtonLabel), findsNothing);
        expect(find.text(_longLocalizations.copyButtonLabel), findsNothing);
        expect(find.text(_longLocalizations.pasteButtonLabel), findsNothing);
        expect(find.text(_longLocalizations.selectAllButtonLabel), findsOneWidget);
        expect(findOverflowBackButton(), findsOneWidget);
        expect(findOverflowNextButton(), findsNothing);

        // Tap select all to show the full selection menu.
        await tester.tap(find.text(_longLocalizations.selectAllButtonLabel));
        await tester.pumpAndSettle();

        // Only one button fits on each page.
        expect(find.text(_longLocalizations.cutButtonLabel), findsOneWidget);
        expect(find.text(_longLocalizations.copyButtonLabel), findsNothing);
        expect(find.text(_longLocalizations.pasteButtonLabel), findsNothing);
        expect(find.text(_longLocalizations.selectAllButtonLabel), findsNothing);
        expect(findOverflowBackButton(), findsNothing);
        expect(findOverflowNextButton(), findsOneWidget);

        // Tap next to go to the second page.
        await tester.tapAt(tester.getCenter(findOverflowNextButton()));
        await tester.pumpAndSettle();
        expect(find.text(_longLocalizations.cutButtonLabel), findsNothing);
        expect(find.text(_longLocalizations.copyButtonLabel), findsOneWidget);
        expect(find.text(_longLocalizations.pasteButtonLabel), findsNothing);
        expect(find.text(_longLocalizations.selectAllButtonLabel), findsNothing);
        expect(findOverflowBackButton(), findsOneWidget);
        expect(findOverflowNextButton(), findsOneWidget);

        // Tap twice to go to the third page.
        await tester.tapAt(tester.getCenter(findOverflowNextButton()));
        await tester.pumpAndSettle();
        expect(find.text(_longLocalizations.cutButtonLabel), findsNothing);
        expect(find.text(_longLocalizations.copyButtonLabel), findsNothing);
        expect(find.text(_longLocalizations.pasteButtonLabel), findsOneWidget);
        expect(find.text(_longLocalizations.selectAllButtonLabel), findsNothing);
        expect(findOverflowBackButton(), findsOneWidget);
        expect(findOverflowNextButton(), findsOneWidget);

        // Tap back to go to the second page again.
        await tester.tapAt(tester.getCenter(findOverflowBackButton()));
        await tester.pumpAndSettle();
        expect(find.text(_longLocalizations.cutButtonLabel), findsNothing);
        expect(find.text(_longLocalizations.copyButtonLabel), findsOneWidget);
        expect(find.text(_longLocalizations.pasteButtonLabel), findsNothing);
        expect(find.text(_longLocalizations.selectAllButtonLabel), findsNothing);
        expect(findOverflowBackButton(), findsOneWidget);
        expect(findOverflowNextButton(), findsOneWidget);

        // Tap back to go to the first page again.
        await tester.tapAt(tester.getCenter(findOverflowBackButton()));
        await tester.pumpAndSettle();
        expect(find.text(_longLocalizations.cutButtonLabel), findsOneWidget);
        expect(find.text(_longLocalizations.copyButtonLabel), findsNothing);
        expect(find.text(_longLocalizations.pasteButtonLabel), findsNothing);
        expect(find.text(_longLocalizations.selectAllButtonLabel), findsNothing);
        expect(findOverflowBackButton(), findsNothing);
        expect(findOverflowNextButton(), findsOneWidget);
      },
      skip: isBrowser, // [intended] We do not use Flutter-rendered context menu on the Web.
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
    );

    testWidgets(
      'When selecting multiple lines over max lines',
      (WidgetTester tester) async {
        final TextEditingController controller = TextEditingController(
          text: 'abc\ndef\nghi\njkl\nmno\npqr',
        );
        addTearDown(controller.dispose);
        await tester.pumpWidget(
          CupertinoApp(
            home: Directionality(
              textDirection: TextDirection.ltr,
              child: MediaQuery(
                data: const MediaQueryData(size: Size(800.0, 600.0)),
                child: Center(
                  child: CupertinoTextField(
                    autofocus: true,
                    padding: const EdgeInsets.all(8.0),
                    controller: controller,
                    maxLines: 2,
                  ),
                ),
              ),
            ),
          ),
        );

        // This extra pump is so autofocus can propagate to renderEditable.
        await tester.pump();

        // Initially, the menu isn't shown at all.
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsNothing);
        expect(find.text('Select All'), findsNothing);
        expect(findOverflowBackButton(), findsNothing);
        expect(findOverflowNextButton(), findsNothing);

        // Long press on an space to show the selection menu.
        await tester.longPressAt(textOffsetToPosition(tester, 1));
        await tester.pumpAndSettle();
        expect(find.text('Cut'), findsNothing);
        expect(find.text('Copy'), findsNothing);
        expect(find.text('Paste'), findsOneWidget);
        expect(find.text('Select All'), findsOneWidget);
        expect(findOverflowBackButton(), findsNothing);
        expect(findOverflowNextButton(), findsNothing);

        // Tap to select all.
        await tester.tap(find.text('Select All'));
        await tester.pumpAndSettle();

        // Only Cut, Copy, and Paste are shown.
        expect(find.text('Cut'), findsOneWidget);
        expect(find.text('Copy'), findsOneWidget);
        expect(find.text('Paste'), findsOneWidget);
        expect(find.text('Select All'), findsNothing);
        expect(findOverflowBackButton(), findsNothing);
        expect(findOverflowNextButton(), findsNothing);

        // The menu appears at the top of the visible selection.
        final Offset selectionOffset = tester.getTopLeft(
          find.byType(CupertinoTextSelectionToolbarButton).first,
        );
        final Offset textFieldOffset = tester.getTopLeft(find.byType(CupertinoTextField));

        // 7.0 + 44.0 + 8.0 - 8.0 = _kToolbarArrowSize + text_button_height + _kToolbarContentDistance - padding
        expect(selectionOffset.dy + 7.0 + 44.0 + 8.0 - 8.0, equals(textFieldOffset.dy));
      },
      skip: isBrowser, // [intended] the selection menu isn't required by web
      variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
    );
  });

  testWidgets(
    'iOS selection handles scale with rich text (selection style 1)',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const CupertinoApp(
          home: Center(
            child: SelectableText.rich(
              TextSpan(
                children: <InlineSpan>[
                  TextSpan(text: 'abc ', style: TextStyle(fontSize: 100.0)),
                  TextSpan(text: 'def ', style: TextStyle(fontSize: 50.0)),
                  TextSpan(text: 'hij', style: TextStyle(fontSize: 25.0)),
                ],
              ),
            ),
          ),
        ),
      );

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText));
      final EditableTextState editableTextState = tester.state(find.byType(EditableText));
      final TextEditingController controller = editableTextWidget.controller;

      // Double tap to select the second word.
      const int index = 4;
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pumpAndSettle();
      expect(editableTextState.selectionOverlay!.handlesAreVisible, isTrue);
      expect(controller.selection.baseOffset, 4);
      expect(controller.selection.extentOffset, 7);

      // Drag the right handle 2 letters to the right. Placing the end handle on
      // the third word. We use a small offset because the endpoint is on the very
      // corner of the handle.
      final TextSelection selection = controller.selection;
      final RenderEditable renderEditable = findRenderEditable(tester);
      final List<TextSelectionPoint> endpoints = globalize(
        renderEditable.getEndpointsForSelection(selection),
        renderEditable,
      );
      expect(endpoints.length, 2);

      final Offset handlePos = endpoints[1].point + const Offset(1.0, 1.0);
      final Offset newHandlePos = textOffsetToPosition(tester, 11);
      final TestGesture gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      expect(controller.selection.baseOffset, 4);
      expect(controller.selection.extentOffset, 11);

      // Find start and end handles and verify their sizes.
      expect(find.byType(Overlay), findsOneWidget);
      expect(
        find.descendant(of: find.byType(Overlay), matching: find.byType(CustomPaint)),
        findsNWidgets(2),
      );

      final Iterable<RenderBox> handles = tester.renderObjectList(
        find.descendant(of: find.byType(Overlay), matching: find.byType(CustomPaint)),
      );

      // The handle height is determined by the formula:
      // textLineHeight + _kSelectionHandleRadius * 2 - _kSelectionHandleOverlap .
      // The text line height will be the value of the fontSize.
      // The constant _kSelectionHandleRadius has the value of 6.
      // The constant _kSelectionHandleOverlap has the value of 1.5.
      // In the case of the start handle, which is located on the word 'def',
      // 50.0 + 6 * 2 - 1.5 = 60.5 .
      expect(handles.first.size.height, 60.5);
      expect(handles.last.size.height, 35.5);
    },
    skip: isBrowser, // [intended] We do not use Flutter-rendered context menu on the Web.
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'iOS selection handles scale with rich text (selection style 2)',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const CupertinoApp(
          home: Center(
            child: SelectableText.rich(
              TextSpan(
                children: <InlineSpan>[
                  TextSpan(text: 'abc ', style: TextStyle(fontSize: 100.0)),
                  TextSpan(text: 'def ', style: TextStyle(fontSize: 50.0)),
                  TextSpan(text: 'hij', style: TextStyle(fontSize: 25.0)),
                ],
              ),
              selectionHeightStyle: ui.BoxHeightStyle.max,
            ),
          ),
        ),
      );

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText));
      final EditableTextState editableTextState = tester.state(find.byType(EditableText));
      final TextEditingController controller = editableTextWidget.controller;

      // Double tap to select the second word.
      const int index = 4;
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pumpAndSettle();
      expect(editableTextState.selectionOverlay!.handlesAreVisible, isTrue);
      expect(controller.selection.baseOffset, 4);
      expect(controller.selection.extentOffset, 7);

      // Drag the right handle 2 letters to the right. Placing the end handle on
      // the third word. We use a small offset because the endpoint is on the very
      // corner of the handle.
      final TextSelection selection = controller.selection;
      final RenderEditable renderEditable = findRenderEditable(tester);
      final List<TextSelectionPoint> endpoints = globalize(
        renderEditable.getEndpointsForSelection(selection),
        renderEditable,
      );
      expect(endpoints.length, 2);

      final Offset handlePos = endpoints[1].point + const Offset(1.0, 1.0);
      final Offset newHandlePos = textOffsetToPosition(tester, 11);
      final TestGesture gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      expect(controller.selection.baseOffset, 4);
      expect(controller.selection.extentOffset, 11);

      // Find start and end handles and verify their sizes.
      expect(find.byType(Overlay), findsOneWidget);
      expect(
        find.descendant(of: find.byType(Overlay), matching: find.byType(CustomPaint)),
        findsNWidgets(2),
      );

      final Iterable<RenderBox> handles = tester.renderObjectList(
        find.descendant(of: find.byType(Overlay), matching: find.byType(CustomPaint)),
      );

      // The handle height is determined by the formula:
      // textLineHeight + _kSelectionHandleRadius * 2 - _kSelectionHandleOverlap .
      // The text line height will be the value of the fontSize, of the largest word on the line.
      // The constant _kSelectionHandleRadius has the value of 6.
      // The constant _kSelectionHandleOverlap has the value of 1.5.
      // In the case of the start handle, which is located on the word 'def',
      // 100 + 6 * 2 - 1.5 = 110.5 .
      // In this case both selection handles are the same size because the selection
      // height style is set to BoxHeightStyle.max which means that the height of
      // the selection highlight will be the height of the largest word on the line.
      expect(handles.first.size.height, 110.5);
      expect(handles.last.size.height, 110.5);
    },
    skip: isBrowser, // [intended] We do not use Flutter-rendered context menu on the Web.
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'iOS selection handles scale with rich text (grapheme clusters)',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const CupertinoApp(
          home: Center(
            child: SelectableText.rich(
              TextSpan(
                children: <InlineSpan>[
                  TextSpan(text: 'abc ', style: TextStyle(fontSize: 100.0)),
                  TextSpan(text: 'def ', style: TextStyle(fontSize: 50.0)),
                  TextSpan(text: '👨‍👩‍👦 ', style: TextStyle(fontSize: 35.0)),
                  TextSpan(text: 'hij', style: TextStyle(fontSize: 25.0)),
                ],
              ),
            ),
          ),
        ),
      );

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText));
      final EditableTextState editableTextState = tester.state(find.byType(EditableText));
      final TextEditingController controller = editableTextWidget.controller;

      // Double tap to select the second word.
      const int index = 4;
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pumpAndSettle();
      expect(editableTextState.selectionOverlay!.handlesAreVisible, isTrue);
      expect(controller.selection.baseOffset, 4);
      expect(controller.selection.extentOffset, 7);

      // Drag the right handle 2 letters to the right. Placing the end handle on
      // the third word. We use a small offset because the endpoint is on the very
      // corner of the handle.
      final TextSelection selection = controller.selection;
      final RenderEditable renderEditable = findRenderEditable(tester);
      final List<TextSelectionPoint> endpoints = globalize(
        renderEditable.getEndpointsForSelection(selection),
        renderEditable,
      );
      expect(endpoints.length, 2);

      final Offset handlePos = endpoints[1].point + const Offset(1.0, 1.0);
      final Offset newHandlePos = textOffsetToPosition(tester, 16);
      final TestGesture gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      expect(controller.selection.baseOffset, 4);
      expect(controller.selection.extentOffset, 16);

      // Find start and end handles and verify their sizes.
      expect(find.byType(Overlay), findsOneWidget);
      expect(
        find.descendant(of: find.byType(Overlay), matching: find.byType(CustomPaint)),
        findsNWidgets(2),
      );

      final Iterable<RenderBox> handles = tester.renderObjectList(
        find.descendant(of: find.byType(Overlay), matching: find.byType(CustomPaint)),
      );

      // The handle height is determined by the formula:
      // textLineHeight + _kSelectionHandleRadius * 2 - _kSelectionHandleOverlap .
      // The text line height will be the value of the fontSize.
      // The constant _kSelectionHandleRadius has the value of 6.
      // The constant _kSelectionHandleOverlap has the value of 1.5.
      // In the case of the end handle, which is located on the grapheme cluster '👨‍👩‍👦',
      // 35.0 + 6 * 2 - 1.5 = 45.5 .
      expect(handles.first.size.height, 60.5);
      expect(handles.last.size.height, 45.5);
    },
    skip: isBrowser, // [intended] We do not use Flutter-rendered context menu on the Web.
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );

  testWidgets(
    'iOS selection handles scaling falls back to preferredLineHeight when the current frame does not match the previous',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const CupertinoApp(
          home: Center(
            child: SelectableText.rich(
              TextSpan(
                children: <InlineSpan>[
                  TextSpan(text: 'abc', style: TextStyle(fontSize: 40.0)),
                  TextSpan(text: 'def', style: TextStyle(fontSize: 50.0)),
                ],
              ),
            ),
          ),
        ),
      );

      final EditableText editableTextWidget = tester.widget(find.byType(EditableText));
      final EditableTextState editableTextState = tester.state(find.byType(EditableText));
      final TextEditingController controller = editableTextWidget.controller;

      // Double tap to select the second word.
      const int index = 4;
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pump(const Duration(milliseconds: 50));
      await tester.tapAt(textOffsetToPosition(tester, index));
      await tester.pumpAndSettle();
      expect(editableTextState.selectionOverlay!.handlesAreVisible, isTrue);
      expect(controller.selection.baseOffset, 0);
      expect(controller.selection.extentOffset, 6);

      // Drag the right handle 2 letters to the right. Placing the end handle on
      // the third word. We use a small offset because the endpoint is on the very
      // corner of the handle.
      final TextSelection selection = controller.selection;
      final RenderEditable renderEditable = findRenderEditable(tester);
      final List<TextSelectionPoint> endpoints = globalize(
        renderEditable.getEndpointsForSelection(selection),
        renderEditable,
      );
      expect(endpoints.length, 2);

      final Offset handlePos = endpoints[1].point + const Offset(1.0, 1.0);
      final Offset newHandlePos = textOffsetToPosition(tester, 3);
      final TestGesture gesture = await tester.startGesture(handlePos, pointer: 7);
      await tester.pump();
      await gesture.moveTo(newHandlePos);
      await tester.pump();
      await gesture.up();
      await tester.pump();

      expect(controller.selection.baseOffset, 0);
      expect(controller.selection.extentOffset, 3);

      // Find start and end handles and verify their sizes.
      expect(find.byType(Overlay), findsOneWidget);
      expect(
        find.descendant(of: find.byType(Overlay), matching: find.byType(CustomPaint)),
        findsNWidgets(2),
      );

      final Iterable<RenderBox> handles = tester.renderObjectList(
        find.descendant(of: find.byType(Overlay), matching: find.byType(CustomPaint)),
      );

      // The handle height is determined by the formula:
      // textLineHeight + _kSelectionHandleRadius * 2 - _kSelectionHandleOverlap .
      // The text line height will be the value of the fontSize.
      // The constant _kSelectionHandleRadius has the value of 6.
      // The constant _kSelectionHandleOverlap has the value of 1.5.
      // In the case of the start handle, which is located on the word 'abc',
      // 40.0 + 6 * 2 - 1.5 = 50.5 .
      //
      // We are now using the current frames selection and text in order to
      // calculate the start and end handle heights (we fall back to preferredLineHeight
      // when the current frame differs from the previous frame), where previously
      // we would be using a mix of the previous and current frame. This could
      // result in the start and end handle heights being calculated inaccurately
      // if one of the handles falls between two varying text styles.
      expect(handles.first.size.height, 50.5);
      expect(handles.last.size.height, 50.5); // This is 60.5 with the previous frame.
    },
    skip: isBrowser, // [intended] We do not use Flutter-rendered context menu on the Web.
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.iOS}),
  );
}
