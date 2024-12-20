// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

void main() {
  final TestWidgetsFlutterBinding binding = TestWidgetsFlutterBinding.ensureInitialized();

  testWidgets(
    'asserts when built on an unsupported device',
    (WidgetTester tester) async {
      final TextEditingController controller = TextEditingController(text: 'one two three');
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        // By default, MediaQueryData.supportsShowingSystemContextMenu is false.
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: TextField(
                controller: controller,
                contextMenuBuilder: (BuildContext context, EditableTextState editableTextState) {
                  return SystemContextMenu.editableText(editableTextState: editableTextState);
                },
              ),
            ),
          ),
        ),
      );

      await tester.tap(find.byType(TextField));
      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      expect(state.showToolbar(), true);
      await tester.pump();

      expect(tester.takeException(), isAssertionError);
    },
    skip: kIsWeb, // [intended]
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'asserts when built on web',
    (WidgetTester tester) async {
      // Disable the browser context menu so that contextMenuBuilder will be used.
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.contextMenu,
        (MethodCall call) {
          // Just complete successfully, so that BrowserContextMenu thinks that
          // the engine successfully received its call.
          return Future<void>.value();
        },
      );
      await BrowserContextMenu.disableContextMenu();
      addTearDown(() async {
        await BrowserContextMenu.enableContextMenu();
        TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
          SystemChannels.contextMenu,
          null,
        );
      });

      final TextEditingController controller = TextEditingController(text: 'one two three');
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        // By default, MediaQueryData.supportsShowingSystemContextMenu is false.
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: TextField(
                controller: controller,
                contextMenuBuilder: (BuildContext context, EditableTextState editableTextState) {
                  return SystemContextMenu.editableText(editableTextState: editableTextState);
                },
              ),
            ),
          ),
        ),
      );

      await tester.tap(find.byType(TextField));
      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      expect(state.showToolbar(), true);
      await tester.pump();

      expect(tester.takeException(), isAssertionError);
    },
    skip: !kIsWeb, // [intended]
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );

  testWidgets(
    'can be shown and hidden like a normal context menu',
    (WidgetTester tester) async {
      final TextEditingController controller = TextEditingController(text: 'one two three');
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        Builder(
          builder: (BuildContext context) {
            final MediaQueryData mediaQueryData = MediaQuery.of(context);
            return MediaQuery(
              data: mediaQueryData.copyWith(supportsShowingSystemContextMenu: true),
              child: MaterialApp(
                home: Scaffold(
                  body: Center(
                    child: TextField(
                      controller: controller,
                      contextMenuBuilder: (
                        BuildContext context,
                        EditableTextState editableTextState,
                      ) {
                        return SystemContextMenu.editableText(editableTextState: editableTextState);
                      },
                    ),
                  ),
                ),
              ),
            );
          },
        ),
      );

      expect(find.byType(SystemContextMenu), findsNothing);

      await tester.tap(find.byType(TextField));
      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      expect(state.showToolbar(), true);
      await tester.pump();
      expect(find.byType(SystemContextMenu), findsOneWidget);

      state.hideToolbar();
      await tester.pump();
      expect(find.byType(SystemContextMenu), findsNothing);
    },
    skip: kIsWeb, // [intended]
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );

  testWidgets(
    'can be updated.',
    (WidgetTester tester) async {
      final List<Map<String, double>> targetRects = <Map<String, double>>[];
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.platform,
        (MethodCall methodCall) async {
          if (methodCall.method == 'ContextMenu.showSystemContextMenu') {
            final Map<String, dynamic> arguments = methodCall.arguments as Map<String, dynamic>;
            final Map<String, dynamic> untypedTargetRect =
                arguments['targetRect'] as Map<String, dynamic>;
            final Map<String, double> lastTargetRect = untypedTargetRect.map((
              String key,
              dynamic value,
            ) {
              return MapEntry<String, double>(key, value as double);
            });
            targetRects.add(lastTargetRect);
          }
          return;
        },
      );
      addTearDown(() {
        TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
          SystemChannels.platform,
          null,
        );
      });

      final TextEditingController controller = TextEditingController(text: 'one two three');
      addTearDown(controller.dispose);
      await tester.pumpWidget(
        Builder(
          builder: (BuildContext context) {
            final MediaQueryData mediaQueryData = MediaQuery.of(context);
            return MediaQuery(
              data: mediaQueryData.copyWith(supportsShowingSystemContextMenu: true),
              child: MaterialApp(
                home: Scaffold(
                  body: Center(
                    child: TextField(
                      controller: controller,
                      contextMenuBuilder: (
                        BuildContext context,
                        EditableTextState editableTextState,
                      ) {
                        return SystemContextMenu.editableText(editableTextState: editableTextState);
                      },
                    ),
                  ),
                ),
              ),
            );
          },
        ),
      );

      expect(targetRects, isEmpty);

      await tester.tap(find.byType(TextField));
      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      expect(state.showToolbar(), true);
      await tester.pump();

      expect(targetRects, hasLength(1));
      expect(targetRects.last, containsPair('width', 0.0));

      controller.selection = const TextSelection(baseOffset: 4, extentOffset: 7);
      await tester.pumpAndSettle();

      expect(targetRects, hasLength(2));
      expect(targetRects.last['width'], greaterThan(0.0));
    },
    skip: kIsWeb, // [intended]
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );

  testWidgets(
    'can be rebuilt',
    (WidgetTester tester) async {
      final TextEditingController controller = TextEditingController(text: 'one two three');
      addTearDown(controller.dispose);
      late StateSetter setState;
      await tester.pumpWidget(
        Builder(
          builder: (BuildContext context) {
            final MediaQueryData mediaQueryData = MediaQuery.of(context);
            return MediaQuery(
              data: mediaQueryData.copyWith(supportsShowingSystemContextMenu: true),
              child: MaterialApp(
                home: Scaffold(
                  body: Center(
                    child: StatefulBuilder(
                      builder: (BuildContext context, StateSetter localSetState) {
                        setState = localSetState;
                        return TextField(
                          controller: controller,
                          contextMenuBuilder: (
                            BuildContext context,
                            EditableTextState editableTextState,
                          ) {
                            return SystemContextMenu.editableText(
                              editableTextState: editableTextState,
                            );
                          },
                        );
                      },
                    ),
                  ),
                ),
              ),
            );
          },
        ),
      );

      await tester.tap(find.byType(TextField));
      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      expect(state.showToolbar(), true);
      await tester.pump();

      setState(() {});
      await tester.pumpAndSettle();

      expect(tester.takeException(), isNull);
    },
    skip: kIsWeb, // [intended]
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );

  testWidgets(
    'can handle multiple instances',
    (WidgetTester tester) async {
      final TextEditingController controller1 = TextEditingController(text: 'one two three');
      addTearDown(controller1.dispose);
      final TextEditingController controller2 = TextEditingController(text: 'four five six');
      addTearDown(controller2.dispose);
      final GlobalKey field1Key = GlobalKey();
      final GlobalKey field2Key = GlobalKey();
      final GlobalKey menu1Key = GlobalKey();
      final GlobalKey menu2Key = GlobalKey();
      await tester.pumpWidget(
        Builder(
          builder: (BuildContext context) {
            final MediaQueryData mediaQueryData = MediaQuery.of(context);
            return MediaQuery(
              data: mediaQueryData.copyWith(supportsShowingSystemContextMenu: true),
              child: MaterialApp(
                home: Scaffold(
                  body: Center(
                    child: Column(
                      children: <Widget>[
                        TextField(
                          key: field1Key,
                          controller: controller1,
                          contextMenuBuilder: (
                            BuildContext context,
                            EditableTextState editableTextState,
                          ) {
                            return SystemContextMenu.editableText(
                              key: menu1Key,
                              editableTextState: editableTextState,
                            );
                          },
                        ),
                        TextField(
                          key: field2Key,
                          controller: controller2,
                          contextMenuBuilder: (
                            BuildContext context,
                            EditableTextState editableTextState,
                          ) {
                            return SystemContextMenu.editableText(
                              key: menu2Key,
                              editableTextState: editableTextState,
                            );
                          },
                        ),
                      ],
                    ),
                  ),
                ),
              ),
            );
          },
        ),
      );

      expect(find.byType(SystemContextMenu), findsNothing);

      await tester.tap(find.byKey(field1Key));
      final EditableTextState state1 = tester.state<EditableTextState>(
        find.descendant(of: find.byKey(field1Key), matching: find.byType(EditableText)),
      );
      expect(state1.showToolbar(), true);
      await tester.pump();
      expect(find.byKey(menu1Key), findsOneWidget);
      expect(find.byKey(menu2Key), findsNothing);

      // In a real app, this message is sent by iOS when the user taps anywhere
      // outside of the system context menu.
      final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
        'method': 'ContextMenu.onDismissSystemContextMenu',
      });
      await binding.defaultBinaryMessenger.handlePlatformMessage(
        'flutter/platform',
        messageBytes,
        (ByteData? data) {},
      );
      await tester.pump();
      expect(find.byType(SystemContextMenu), findsNothing);

      await tester.tap(find.byKey(field2Key));
      final EditableTextState state2 = tester.state<EditableTextState>(
        find.descendant(of: find.byKey(field2Key), matching: find.byType(EditableText)),
      );
      expect(state2.showToolbar(), true);
      await tester.pump();
      expect(find.byKey(menu1Key), findsNothing);
      expect(find.byKey(menu2Key), findsOneWidget);
    },
    skip: kIsWeb, // [intended]
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );

  testWidgets(
    'asserts when built with no text input connection',
    experimentalLeakTesting:
        LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
    (WidgetTester tester) async {
      SystemContextMenu? systemContextMenu;
      late StateSetter setState;
      await tester.pumpWidget(
        Builder(
          builder: (BuildContext context) {
            final MediaQueryData mediaQueryData = MediaQuery.of(context);
            return MediaQuery(
              data: mediaQueryData.copyWith(supportsShowingSystemContextMenu: true),
              child: MaterialApp(
                home: Scaffold(
                  body: StatefulBuilder(
                    builder: (BuildContext context, StateSetter localSetState) {
                      setState = localSetState;
                      return Column(
                        children: <Widget>[
                          const TextField(),
                          if (systemContextMenu != null) systemContextMenu!,
                        ],
                      );
                    },
                  ),
                ),
              ),
            );
          },
        ),
      );

      // No SystemContextMenu yet, so no assertion error.
      expect(tester.takeException(), isNull);

      // Add the SystemContextMenu and receive an assertion since there is no
      // active text input connection.
      setState(() {
        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
        systemContextMenu = SystemContextMenu.editableText(editableTextState: state);
      });

      final FlutterExceptionHandler? oldHandler = FlutterError.onError;
      dynamic exception;
      FlutterError.onError = (FlutterErrorDetails details) {
        exception ??= details.exception;
      };
      addTearDown(() {
        FlutterError.onError = oldHandler;
      });

      await tester.pump();
      expect(exception, isAssertionError);
      expect(exception.toString(), contains('only be shown for an active text input connection'));
    },
    skip: kIsWeb, // [intended]
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );

  testWidgets(
    'does not assert when built with an active text input connection',
    (WidgetTester tester) async {
      SystemContextMenu? systemContextMenu;
      late StateSetter setState;
      await tester.pumpWidget(
        Builder(
          builder: (BuildContext context) {
            final MediaQueryData mediaQueryData = MediaQuery.of(context);
            return MediaQuery(
              data: mediaQueryData.copyWith(supportsShowingSystemContextMenu: true),
              child: MaterialApp(
                home: Scaffold(
                  body: StatefulBuilder(
                    builder: (BuildContext context, StateSetter localSetState) {
                      setState = localSetState;
                      return Column(
                        children: <Widget>[
                          const TextField(),
                          if (systemContextMenu != null) systemContextMenu!,
                        ],
                      );
                    },
                  ),
                ),
              ),
            );
          },
        ),
      );

      // No SystemContextMenu yet, so no assertion error.
      expect(tester.takeException(), isNull);

      // Tap the field to open a text input connection.
      await tester.tap(find.byType(TextField));
      await tester.pump();

      // Add the SystemContextMenu and expect no error.
      setState(() {
        final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
        systemContextMenu = SystemContextMenu.editableText(editableTextState: state);
      });

      final FlutterExceptionHandler? oldHandler = FlutterError.onError;
      dynamic exception;
      FlutterError.onError = (FlutterErrorDetails details) {
        exception ??= details.exception;
      };
      addTearDown(() {
        FlutterError.onError = oldHandler;
      });

      await tester.pump();
      expect(exception, isNull);
    },
    skip: kIsWeb, // [intended]
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );
}
