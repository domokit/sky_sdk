// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import '../system_context_menu_utils.dart';

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
    'can customize the menu items',
    (WidgetTester tester) async {
      final List<List<IOSSystemContextMenuItemData>> itemsReceived =
          <List<IOSSystemContextMenuItemData>>[];
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.platform,
        (MethodCall methodCall) async {
          switch (methodCall.method) {
            case 'ContextMenu.showSystemContextMenu':
              final Map<String, dynamic> arguments = methodCall.arguments as Map<String, dynamic>;
              final List<dynamic> untypedItems = arguments['items'] as List<dynamic>;
              final List<IOSSystemContextMenuItemData> lastItems =
                  untypedItems.map((dynamic value) {
                    final Map<String, dynamic> itemJson = value as Map<String, dynamic>;
                    return systemContextMenuItemDataFromJson(itemJson);
                  }).toList();
              itemsReceived.add(lastItems);
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

      const List<IOSSystemContextMenuItem> items1 = <IOSSystemContextMenuItem>[
        IOSSystemContextMenuItemCopy(),
        IOSSystemContextMenuItemShare(title: 'My Share Title'),
      ];
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
                        return SystemContextMenu.editableText(
                          editableTextState: editableTextState,
                          items: items1,
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

      expect(find.byType(SystemContextMenu), findsNothing);
      expect(itemsReceived, hasLength(0));

      await tester.tap(find.byType(TextField));
      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      expect(state.showToolbar(), true);
      await tester.pump();
      expect(find.byType(SystemContextMenu), findsOneWidget);

      expect(itemsReceived, hasLength(1));
      expect(itemsReceived.last, hasLength(items1.length));
      expect(itemsReceived.last[0], equals(const IOSSystemContextMenuItemDataCopy()));
      expect(
        itemsReceived.last[1],
        equals(const IOSSystemContextMenuItemDataShare(title: 'My Share Title')),
      );

      state.hideToolbar();
      await tester.pump();
      expect(find.byType(SystemContextMenu), findsNothing);
    },
    skip: kIsWeb, // [intended]
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );

  testWidgets(
    "passing empty items builds the widget but doesn't show the system context menu",
    (WidgetTester tester) async {
      final List<List<IOSSystemContextMenuItemData>> itemsReceived =
          <List<IOSSystemContextMenuItemData>>[];
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.platform,
        (MethodCall methodCall) async {
          switch (methodCall.method) {
            case 'ContextMenu.showSystemContextMenu':
              final Map<String, dynamic> arguments = methodCall.arguments as Map<String, dynamic>;
              final List<dynamic> untypedItems = arguments['items'] as List<dynamic>;
              final List<IOSSystemContextMenuItemData> lastItems =
                  untypedItems.map((dynamic value) {
                    final Map<String, dynamic> itemJson = value as Map<String, dynamic>;
                    return systemContextMenuItemDataFromJson(itemJson);
                  }).toList();
              itemsReceived.add(lastItems);
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

      const List<IOSSystemContextMenuItem> items1 = <IOSSystemContextMenuItem>[];
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
                        return SystemContextMenu.editableText(
                          editableTextState: editableTextState,
                          items: items1,
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

      expect(tester.takeException(), isNull);

      expect(find.byType(SystemContextMenu), findsNothing);
      expect(itemsReceived, hasLength(0));

      await tester.tap(find.byType(TextField));
      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      expect(state.showToolbar(), true);
      expect(tester.takeException(), isNull);

      await tester.pump();
      expect(tester.takeException(), isNull);
      expect(find.byType(SystemContextMenu), findsOneWidget);
      expect(itemsReceived, hasLength(0));
    },
    skip: kIsWeb, // [intended]
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );

  testWidgets(
    'items receive a default title',
    (WidgetTester tester) async {
      final List<List<IOSSystemContextMenuItemData>> itemsReceived =
          <List<IOSSystemContextMenuItemData>>[];
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.platform,
        (MethodCall methodCall) async {
          switch (methodCall.method) {
            case 'ContextMenu.showSystemContextMenu':
              final Map<String, dynamic> arguments = methodCall.arguments as Map<String, dynamic>;
              final List<dynamic> untypedItems = arguments['items'] as List<dynamic>;
              final List<IOSSystemContextMenuItemData> lastItems =
                  untypedItems.map((dynamic value) {
                    final Map<String, dynamic> itemJson = value as Map<String, dynamic>;
                    return systemContextMenuItemDataFromJson(itemJson);
                  }).toList();
              itemsReceived.add(lastItems);
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

      const List<IOSSystemContextMenuItem> items1 = <IOSSystemContextMenuItem>[
        // Copy gets no title, it's set by the platform.
        IOSSystemContextMenuItemCopy(),
        // Share could take a title, but if not, it gets a localized default.
        IOSSystemContextMenuItemShare(),
      ];
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
                        return SystemContextMenu.editableText(
                          editableTextState: editableTextState,
                          items: items1,
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

      expect(find.byType(SystemContextMenu), findsNothing);
      expect(itemsReceived, hasLength(0));

      await tester.tap(find.byType(TextField));
      final EditableTextState state = tester.state<EditableTextState>(find.byType(EditableText));
      expect(state.showToolbar(), true);
      await tester.pump();
      expect(find.byType(SystemContextMenu), findsOneWidget);

      expect(itemsReceived, hasLength(1));
      expect(itemsReceived.last, hasLength(items1.length));
      expect(itemsReceived.last[0], equals(const IOSSystemContextMenuItemDataCopy()));
      const WidgetsLocalizations localizations = DefaultWidgetsLocalizations();
      expect(
        itemsReceived.last[1],
        equals(IOSSystemContextMenuItemDataShare(title: localizations.shareButtonLabel)),
      );

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

  test(
    'can get the IOSSystemContextMenuItemData representation of an IOSSystemContextMenuItemCopy',
    () {
      const IOSSystemContextMenuItemCopy item = IOSSystemContextMenuItemCopy();
      const WidgetsLocalizations localizations = DefaultWidgetsLocalizations();
      expect(item.getData(localizations), const IOSSystemContextMenuItemDataCopy());
    },
  );

  test(
    'can get the IOSSystemContextMenuItemData representation of an IOSSystemContextMenuItemCut',
    () {
      const IOSSystemContextMenuItemCut item = IOSSystemContextMenuItemCut();
      const WidgetsLocalizations localizations = DefaultWidgetsLocalizations();
      expect(item.getData(localizations), const IOSSystemContextMenuItemDataCut());
    },
  );

  test(
    'can get the IOSSystemContextMenuItemData representation of an IOSSystemContextMenuItemPaste',
    () {
      const IOSSystemContextMenuItemPaste item = IOSSystemContextMenuItemPaste();
      const WidgetsLocalizations localizations = DefaultWidgetsLocalizations();
      expect(item.getData(localizations), const IOSSystemContextMenuItemDataPaste());
    },
  );

  test(
    'can get the IOSSystemContextMenuItemData representation of an IOSSystemContextMenuItemSelectAll',
    () {
      const IOSSystemContextMenuItemSelectAll item = IOSSystemContextMenuItemSelectAll();
      const WidgetsLocalizations localizations = DefaultWidgetsLocalizations();
      expect(item.getData(localizations), const IOSSystemContextMenuItemDataSelectAll());
    },
  );

  test(
    'can get the IOSSystemContextMenuItemData representation of an IOSSystemContextMenuItemLookUp',
    () {
      const IOSSystemContextMenuItemLookUp item = IOSSystemContextMenuItemLookUp();
      const WidgetsLocalizations localizations = DefaultWidgetsLocalizations();
      expect(
        item.getData(localizations),
        IOSSystemContextMenuItemDataLookUp(title: localizations.lookUpButtonLabel),
      );
    },
  );

  test(
    'can get the IOSSystemContextMenuItemData representation of an IOSSystemContextMenuItemSearchWeb',
    () {
      const IOSSystemContextMenuItemSearchWeb item = IOSSystemContextMenuItemSearchWeb();
      const WidgetsLocalizations localizations = DefaultWidgetsLocalizations();
      expect(
        item.getData(localizations),
        IOSSystemContextMenuItemDataSearchWeb(title: localizations.searchWebButtonLabel),
      );
    },
  );

  test(
    'can get the IOSSystemContextMenuItemData representation of an IOSSystemContextMenuItemShare',
    () {
      const IOSSystemContextMenuItemShare item = IOSSystemContextMenuItemShare();
      const WidgetsLocalizations localizations = DefaultWidgetsLocalizations();
      expect(
        item.getData(localizations),
        IOSSystemContextMenuItemDataShare(title: localizations.shareButtonLabel),
      );
    },
  );
}
