// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import 'clipboard_utils.dart';
import 'editable_text_utils.dart';

void main() {
  final MockClipboard mockClipboard = MockClipboard();
  TestWidgetsFlutterBinding.ensureInitialized().defaultBinaryMessenger.setMockMethodCallHandler(
    SystemChannels.platform,
    mockClipboard.handleMethodCall,
  );

  setUp(() async {
    // Fill the clipboard so that the Paste option is available in the text
    // selection menu.
    await Clipboard.setData(const ClipboardData(text: 'Clipboard data'));
  });

  testWidgets('Hides and shows only a single menu', (WidgetTester tester) async {
    final GlobalKey key1 = GlobalKey();
    final GlobalKey key2 = GlobalKey();
    late final BuildContext context;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext localContext) {
              context = localContext;
              return const SizedBox.shrink();
            },
          ),
        ),
      ),
    );

    expect(find.byKey(key1), findsNothing);
    expect(find.byKey(key2), findsNothing);

    final ContextMenuController controller1 = ContextMenuController();
    await tester.pump();
    expect(find.byKey(key1), findsNothing);
    expect(find.byKey(key2), findsNothing);

    controller1.show(
      context: context,
      contextMenuBuilder: (BuildContext context) {
        return Placeholder(key: key1);
      },
    );
    await tester.pump();

    expect(find.byKey(key1), findsOneWidget);
    expect(find.byKey(key2), findsNothing);

    // Showing the same thing again does nothing and is not an error.
    controller1.show(
      context: context,
      contextMenuBuilder: (BuildContext context) {
        return Placeholder(key: key1);
      },
    );
    await tester.pump();

    expect(tester.takeException(), null);
    expect(find.byKey(key1), findsOneWidget);
    expect(find.byKey(key2), findsNothing);

    // Showing a new menu hides the first.
    final ContextMenuController controller2 = ContextMenuController();
    controller2.show(
      context: context,
      contextMenuBuilder: (BuildContext context) {
        return Placeholder(key: key2);
      },
    );
    await tester.pump();

    expect(find.byKey(key1), findsNothing);
    expect(find.byKey(key2), findsOneWidget);

    controller2.remove();
    await tester.pump();

    expect(find.byKey(key1), findsNothing);
    expect(find.byKey(key2), findsNothing);
  });

  testWidgets('A menu can be hidden and then reshown', (WidgetTester tester) async {
    final GlobalKey key1 = GlobalKey();
    late final BuildContext context;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext localContext) {
              context = localContext;
              return const SizedBox.shrink();
            },
          ),
        ),
      ),
    );

    expect(find.byKey(key1), findsNothing);

    final ContextMenuController controller = ContextMenuController();
    addTearDown(controller.remove);

    // Instantiating the controller does not shown it.
    await tester.pump();
    expect(find.byKey(key1), findsNothing);

    controller.show(
      context: context,
      contextMenuBuilder: (BuildContext context) {
        return Placeholder(key: key1);
      },
    );
    await tester.pump();

    expect(find.byKey(key1), findsOneWidget);

    controller.remove();
    await tester.pump();

    expect(find.byKey(key1), findsNothing);

    controller.show(
      context: context,
      contextMenuBuilder: (BuildContext context) {
        return Placeholder(key: key1);
      },
    );
    await tester.pump();

    expect(find.byKey(key1), findsOneWidget);
  });

  testWidgets('markNeedsBuild causes the builder to update', (WidgetTester tester) async {
    int buildCount = 0;
    late final BuildContext context;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Builder(
            builder: (BuildContext localContext) {
              context = localContext;
              return const SizedBox.shrink();
            },
          ),
        ),
      ),
    );

    final ContextMenuController controller = ContextMenuController();
    controller.show(
      context: context,
      contextMenuBuilder: (BuildContext context) {
        buildCount++;
        return const Placeholder();
      },
    );
    expect(buildCount, 0);
    await tester.pump();
    expect(buildCount, 1);

    controller.markNeedsBuild();
    expect(buildCount, 1);
    await tester.pump();
    expect(buildCount, 2);

    controller.remove();
  });

  testWidgets(
    'Calling show when a built-in widget is already showing its context menu hides the built-in menu',
    (WidgetTester tester) async {
      final GlobalKey builtInKey = GlobalKey();
      final GlobalKey directKey = GlobalKey();
      late final BuildContext context;

      final TextEditingController textEditingController = TextEditingController();
      addTearDown(textEditingController.dispose);

      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Builder(
              builder: (BuildContext localContext) {
                context = localContext;
                return EditableText(
                  controller: textEditingController,
                  backgroundCursorColor: Colors.grey,
                  focusNode: focusNode,
                  style: const TextStyle(),
                  cursorColor: Colors.red,
                  selectionControls: materialTextSelectionHandleControls,
                  contextMenuBuilder: (BuildContext context, EditableTextState editableTextState) {
                    return Placeholder(key: builtInKey);
                  },
                );
              },
            ),
          ),
        ),
      );

      expect(find.byKey(builtInKey), findsNothing);
      expect(find.byKey(directKey), findsNothing);

      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));

      await tester.tapAt(textOffsetToPosition(tester, 0));
      await tester.pump();
      expect(state.showToolbar(), true);
      await tester.pump();

      expect(find.byKey(builtInKey), findsOneWidget);
      expect(find.byKey(directKey), findsNothing);

      final ContextMenuController controller = ContextMenuController();
      controller.show(
        context: context,
        contextMenuBuilder: (BuildContext context) {
          return Placeholder(key: directKey);
        },
      );
      await tester.pump();

      expect(find.byKey(builtInKey), findsNothing);
      expect(find.byKey(directKey), findsOneWidget);
      expect(controller.isShown, isTrue);

      // And showing the built-in menu hides the directly shown menu.
      expect(state.showToolbar(), isTrue);
      await tester.pump();

      expect(find.byKey(builtInKey), findsOneWidget);
      expect(find.byKey(directKey), findsNothing);
      expect(controller.isShown, isFalse);

      // Calling remove on the hidden ContextMenuController does not hide the
      // built-in menu.
      controller.remove();
      await tester.pump();

      expect(find.byKey(builtInKey), findsOneWidget);
      expect(find.byKey(directKey), findsNothing);
      expect(controller.isShown, isFalse);

      state.hideToolbar();
      await tester.pump();
      expect(find.byKey(builtInKey), findsNothing);
      expect(find.byKey(directKey), findsNothing);
      expect(controller.isShown, isFalse);
    },
    // [intended] no Flutter-drawn text selection toolbar on web.
    skip: isContextMenuProvidedByPlatform,
  );
}
