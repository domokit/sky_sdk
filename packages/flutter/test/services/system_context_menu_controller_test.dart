// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import '../system_context_menu_utils.dart';
import '../widgets/clipboard_utils.dart';
import 'text_input_utils.dart';

void main() {
  final TestWidgetsFlutterBinding binding = TestWidgetsFlutterBinding.ensureInitialized();

  setUp(() async {
    final MockClipboard mockClipboard = MockClipboard();
    TestWidgetsFlutterBinding.ensureInitialized().defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      mockClipboard.handleMethodCall,
    );
    // Fill the clipboard so that the Paste option is available in the text
    // selection menu.
    await Clipboard.setData(const ClipboardData(text: 'Clipboard data'));
  });

  tearDown(() {
    TestWidgetsFlutterBinding.ensureInitialized().defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      null,
    );
  });

  test('showing and hiding one controller', () {
    // Create an active connection, which is required to show the system menu.
    final FakeTextInputClient client = FakeTextInputClient(const TextEditingValue(text: 'test1'));
    final TextInputConnection connection = TextInput.attach(client, client.configuration);
    addTearDown(() {
      connection.close();
    });

    final List<Map<String, double>> targetRects = <Map<String, double>>[];
    int hideCount = 0;
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      (MethodCall methodCall) async {
        switch (methodCall.method) {
          case 'ContextMenu.showSystemContextMenu':
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
          case 'ContextMenu.hideSystemContextMenu':
            hideCount += 1;
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

    final SystemContextMenuController controller = SystemContextMenuController();
    addTearDown(() {
      controller.dispose();
    });

    expect(targetRects, isEmpty);
    expect(hideCount, 0);
    expect(controller.isVisible, isFalse);

    // Showing calls the platform.
    const Rect rect1 = Rect.fromLTWH(0.0, 0.0, 100.0, 100.0);
    final List<IOSSystemContextMenuItemData> items = <IOSSystemContextMenuItemData>[
      const IOSSystemContextMenuItemDataCopy(),
    ];
    controller.showWithItems(rect1, items);
    expect(targetRects, hasLength(1));
    expect(targetRects.last['x'], rect1.left);
    expect(targetRects.last['y'], rect1.top);
    expect(targetRects.last['width'], rect1.width);
    expect(targetRects.last['height'], rect1.height);

    // Showing the same thing again does nothing.
    controller.showWithItems(rect1, items);
    expect(controller.isVisible, isTrue);
    expect(targetRects, hasLength(1));

    // Showing a new rect calls the platform.
    const Rect rect2 = Rect.fromLTWH(1.0, 1.0, 200.0, 200.0);
    controller.showWithItems(rect2, items);
    expect(targetRects, hasLength(2));
    expect(targetRects.last['x'], rect2.left);
    expect(targetRects.last['y'], rect2.top);
    expect(targetRects.last['width'], rect2.width);
    expect(targetRects.last['height'], rect2.height);

    // Hiding calls the platform.
    controller.hide();
    expect(controller.isVisible, isFalse);
    expect(hideCount, 1);

    // Hiding again does nothing.
    controller.hide();
    expect(controller.isVisible, isFalse);
    expect(hideCount, 1);

    // Showing the last shown rect calls the platform.
    controller.showWithItems(rect2, items);
    expect(controller.isVisible, isTrue);
    expect(targetRects, hasLength(3));
    expect(targetRects.last['x'], rect2.left);
    expect(targetRects.last['y'], rect2.top);
    expect(targetRects.last['width'], rect2.width);
    expect(targetRects.last['height'], rect2.height);

    controller.hide();
    expect(controller.isVisible, isFalse);
    expect(hideCount, 2);
  });

  test('the system can hide the menu with handleSystemHide', () async {
    // Create an active connection, which is required to show the system menu.
    final FakeTextInputClient client = FakeTextInputClient(const TextEditingValue(text: 'test1'));
    final TextInputConnection connection = TextInput.attach(client, client.configuration);
    addTearDown(() {
      connection.close();
    });

    final List<Map<String, double>> targetRects = <Map<String, double>>[];
    int hideCount = 0;
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      (MethodCall methodCall) async {
        switch (methodCall.method) {
          case 'ContextMenu.showSystemContextMenu':
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
          case 'ContextMenu.hideSystemContextMenu':
            hideCount += 1;
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

    int systemHideCount = 0;
    final SystemContextMenuController controller = SystemContextMenuController(
      onSystemHide: () {
        systemHideCount += 1;
      },
    );
    addTearDown(() {
      controller.dispose();
    });

    expect(controller.isVisible, isFalse);
    expect(targetRects, isEmpty);
    expect(hideCount, 0);
    expect(systemHideCount, 0);

    // Showing calls the platform.
    const Rect rect1 = Rect.fromLTWH(0.0, 0.0, 100.0, 100.0);
    final List<IOSSystemContextMenuItemData> items = <IOSSystemContextMenuItemData>[
      const IOSSystemContextMenuItemDataCopy(),
    ];
    controller.showWithItems(rect1, items);
    expect(controller.isVisible, isTrue);
    expect(targetRects, hasLength(1));
    expect(targetRects.last['x'], rect1.left);
    expect(targetRects.last['y'], rect1.top);
    expect(targetRects.last['width'], rect1.width);
    expect(targetRects.last['height'], rect1.height);

    // If the system hides the menu, onSystemHide is called.
    final ByteData? messageBytes = const JSONMessageCodec().encodeMessage(<String, dynamic>{
      'method': 'ContextMenu.onDismissSystemContextMenu',
    });
    await binding.defaultBinaryMessenger.handlePlatformMessage(
      'flutter/platform',
      messageBytes,
      (ByteData? data) {},
    );
    expect(controller.isVisible, isFalse);
    expect(hideCount, 0);
    expect(systemHideCount, 1);

    // Hiding does not call the platform, since the menu was already hidden.
    controller.hide();
    expect(controller.isVisible, isFalse);
    expect(hideCount, 0);
  });

  test('showing a second controller while one is visible is an error', () {
    // Create an active connection, which is required to show the system menu.
    final FakeTextInputClient client = FakeTextInputClient(const TextEditingValue(text: 'test1'));
    final TextInputConnection connection = TextInput.attach(client, client.configuration);
    addTearDown(() {
      connection.close();
    });

    final SystemContextMenuController controller1 = SystemContextMenuController();
    addTearDown(() {
      controller1.dispose();
    });
    expect(controller1.isVisible, isFalse);
    const Rect rect1 = Rect.fromLTWH(0.0, 0.0, 100.0, 100.0);
    final List<IOSSystemContextMenuItemData> items = <IOSSystemContextMenuItemData>[
      const IOSSystemContextMenuItemDataCopy(),
    ];
    expect(() {
      controller1.showWithItems(rect1, items);
    }, isNot(throwsAssertionError));
    expect(controller1.isVisible, isTrue);

    final SystemContextMenuController controller2 = SystemContextMenuController();
    addTearDown(() {
      controller2.dispose();
    });
    expect(controller1.isVisible, isTrue);
    expect(controller2.isVisible, isFalse);
    const Rect rect2 = Rect.fromLTWH(1.0, 1.0, 200.0, 200.0);
    expect(() {
      controller2.showWithItems(rect2, items);
    }, throwsAssertionError);
    expect(controller1.isVisible, isTrue);
    expect(controller2.isVisible, isFalse);

    controller1.hide();
    expect(controller1.isVisible, isFalse);
    expect(controller2.isVisible, isFalse);
  });

  test('showing and hiding two controllers', () {
    // Create an active connection, which is required to show the system menu.
    final FakeTextInputClient client = FakeTextInputClient(const TextEditingValue(text: 'test1'));
    final TextInputConnection connection = TextInput.attach(client, client.configuration);
    addTearDown(() {
      connection.close();
    });

    final List<Map<String, double>> targetRects = <Map<String, double>>[];
    int hideCount = 0;
    TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.platform,
      (MethodCall methodCall) async {
        switch (methodCall.method) {
          case 'ContextMenu.showSystemContextMenu':
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
          case 'ContextMenu.hideSystemContextMenu':
            hideCount += 1;
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

    final SystemContextMenuController controller1 = SystemContextMenuController();
    addTearDown(() {
      controller1.dispose();
    });

    expect(controller1.isVisible, isFalse);
    expect(targetRects, isEmpty);
    expect(hideCount, 0);

    // Showing calls the platform.
    const Rect rect1 = Rect.fromLTWH(0.0, 0.0, 100.0, 100.0);
    final List<IOSSystemContextMenuItemData> items = <IOSSystemContextMenuItemData>[
      const IOSSystemContextMenuItemDataCopy(),
    ];
    controller1.showWithItems(rect1, items);
    expect(controller1.isVisible, isTrue);
    expect(targetRects, hasLength(1));
    expect(targetRects.last['x'], rect1.left);

    // Hiding calls the platform.
    controller1.hide();
    expect(controller1.isVisible, isFalse);
    expect(hideCount, 1);

    // Showing a new controller calls the platform.
    final SystemContextMenuController controller2 = SystemContextMenuController();
    addTearDown(() {
      controller2.dispose();
    });
    expect(controller2.isVisible, isFalse);
    const Rect rect2 = Rect.fromLTWH(1.0, 1.0, 200.0, 200.0);
    controller2.showWithItems(rect2, items);
    expect(controller1.isVisible, isFalse);
    expect(controller2.isVisible, isTrue);
    expect(targetRects, hasLength(2));
    expect(targetRects.last['x'], rect2.left);
    expect(targetRects.last['y'], rect2.top);
    expect(targetRects.last['width'], rect2.width);
    expect(targetRects.last['height'], rect2.height);

    // Hiding the old controller does nothing.
    controller1.hide();
    expect(controller1.isVisible, isFalse);
    expect(controller2.isVisible, isTrue);
    expect(hideCount, 1);

    // Hiding the new controller calls the platform.
    controller2.hide();
    expect(controller1.isVisible, isFalse);
    expect(controller2.isVisible, isFalse);
    expect(hideCount, 2);
  });

  test('showing a controller with custom items', () {
    // Create an active connection, which is required to show the system menu.
    final FakeTextInputClient client = FakeTextInputClient(const TextEditingValue(text: 'test1'));
    final TextInputConnection connection = TextInput.attach(client, client.configuration);
    addTearDown(() {
      connection.close();
    });

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

    final SystemContextMenuController controller = SystemContextMenuController();
    addTearDown(() {
      controller.dispose();
    });

    expect(controller.isVisible, isFalse);

    // Showing calls the platform.
    const Rect rect = Rect.fromLTWH(0.0, 0.0, 100.0, 100.0);
    final List<IOSSystemContextMenuItemData> items1 = <IOSSystemContextMenuItemData>[
      const IOSSystemContextMenuItemDataCut(),
      const IOSSystemContextMenuItemDataCopy(),
      const IOSSystemContextMenuItemDataPaste(),
      const IOSSystemContextMenuItemDataSelectAll(),
      const IOSSystemContextMenuItemDataSearchWeb(title: 'Special Search'),
      // TODO(justinmc): Support the "custom" item type.
      // https://github.com/flutter/flutter/issues/103163
    ];

    controller.showWithItems(rect, items1);
    expect(controller.isVisible, isTrue);
    expect(itemsReceived, hasLength(1));
    expect(itemsReceived.last, hasLength(items1.length));
    expect(itemsReceived.last, equals(items1));

    // Showing the same thing again does nothing.
    controller.showWithItems(rect, items1);
    expect(controller.isVisible, isTrue);
    expect(itemsReceived, hasLength(1));

    // Showing new items calls the platform.
    final List<IOSSystemContextMenuItemData> items2 = <IOSSystemContextMenuItemData>[
      const IOSSystemContextMenuItemDataCut(),
    ];
    controller.showWithItems(rect, items2);
    expect(controller.isVisible, isTrue);
    expect(itemsReceived, hasLength(2));
    expect(itemsReceived.last, hasLength(items2.length));
    expect(itemsReceived.last, equals(items2));

    controller.hide();
    expect(controller.isVisible, isFalse);
  });

  test('showing a controller with empty items', () {
    // Create an active connection, which is required to show the system menu.
    final FakeTextInputClient client = FakeTextInputClient(const TextEditingValue(text: 'test1'));
    final TextInputConnection connection = TextInput.attach(client, client.configuration);
    addTearDown(() {
      connection.close();
    });

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

    final SystemContextMenuController controller = SystemContextMenuController();
    addTearDown(() {
      controller.dispose();
    });

    expect(controller.isVisible, isFalse);

    const Rect rect = Rect.fromLTWH(0.0, 0.0, 100.0, 100.0);
    final List<IOSSystemContextMenuItemData> items = <IOSSystemContextMenuItemData>[];

    expect(() {
      controller.showWithItems(rect, items);
    }, throwsAssertionError);
    expect(controller.isVisible, isFalse);
    expect(itemsReceived, hasLength(0));
  });

  testWidgets('showing a controller for an EditableText', (WidgetTester tester) async {
    final TextEditingController textEditingController = TextEditingController(text: 'test');
    final FocusNode focusNode = FocusNode();
    final GlobalKey<EditableTextState> key = GlobalKey<EditableTextState>();
    late final WidgetsLocalizations localizations;
    addTearDown(() {
      textEditingController.dispose();
      focusNode.dispose();
    });

    await tester.pumpWidget(
      MaterialApp(
        home: Align(
          alignment: Alignment.topLeft,
          child: SizedBox(
            width: 400,
            child: Builder(
              builder: (BuildContext context) {
                localizations = WidgetsLocalizations.of(context);
                return EditableText(
                  key: key,
                  maxLines: 10,
                  controller: textEditingController,
                  showSelectionHandles: true,
                  autofocus: true,
                  focusNode: focusNode,
                  style: Typography.material2018().black.titleMedium!,
                  cursorColor: Colors.blue,
                  backgroundCursorColor: Colors.grey,
                  keyboardType: TextInputType.text,
                  textAlign: TextAlign.right,
                  selectionControls: materialTextSelectionHandleControls,
                );
              },
            ),
          ),
        ),
      ),
    );

    final EditableTextState editableTextState = tester.state<EditableTextState>(
      find.byType(EditableText),
    );
    final List<IOSSystemContextMenuItem> defaultItems = SystemContextMenu.getDefaultItems(
      editableTextState,
    );
    expect(defaultItems, hasLength(2));
    expect(defaultItems[1], const IOSSystemContextMenuItemSelectAll());
    expect(defaultItems.first, const IOSSystemContextMenuItemPaste());

    final (startGlyphHeight: double startGlyphHeight, endGlyphHeight: double endGlyphHeight) =
        editableTextState.getGlyphHeights();
    final Rect anchor = TextSelectionToolbarAnchors.getSelectionRect(
      editableTextState.renderEditable,
      startGlyphHeight,
      endGlyphHeight,
      editableTextState.renderEditable.getEndpointsForSelection(
        editableTextState.textEditingValue.selection,
      ),
    );
    final List<IOSSystemContextMenuItemData> defaultItemDatas =
        defaultItems.map((IOSSystemContextMenuItem item) => item.getData(localizations)).toList();

    expect(defaultItemDatas, isNotEmpty);

    final SystemContextMenuController systemContextMenuController = SystemContextMenuController();
    addTearDown(() {
      systemContextMenuController.dispose();
    });

    expect(systemContextMenuController.isVisible, isFalse);
    systemContextMenuController.showWithItems(anchor, defaultItemDatas);
    expect(systemContextMenuController.isVisible, isTrue);
  });
}
