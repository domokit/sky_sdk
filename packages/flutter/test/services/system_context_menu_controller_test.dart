// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import './text_input_utils.dart';

void main() {
  final TestWidgetsFlutterBinding binding = TestWidgetsFlutterBinding.ensureInitialized();

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

    // Showing calls the platform.
    const Rect rect1 = Rect.fromLTWH(0.0, 0.0, 100.0, 100.0);
    controller.show(rect1);
    expect(targetRects, hasLength(1));
    expect(targetRects.last['x'], rect1.left);
    expect(targetRects.last['y'], rect1.top);
    expect(targetRects.last['width'], rect1.width);
    expect(targetRects.last['height'], rect1.height);

    // Showing the same thing again does nothing.
    controller.show(rect1);
    expect(targetRects, hasLength(1));

    // Showing a new rect calls the platform.
    const Rect rect2 = Rect.fromLTWH(1.0, 1.0, 200.0, 200.0);
    controller.show(rect2);
    expect(targetRects, hasLength(2));
    expect(targetRects.last['x'], rect2.left);
    expect(targetRects.last['y'], rect2.top);
    expect(targetRects.last['width'], rect2.width);
    expect(targetRects.last['height'], rect2.height);

    // Hiding calls the platform.
    controller.hide();
    expect(hideCount, 1);

    // Hiding again does nothing.
    controller.hide();
    expect(hideCount, 1);

    // Showing the last shown rect calls the platform.
    controller.show(rect2);
    expect(targetRects, hasLength(3));
    expect(targetRects.last['x'], rect2.left);
    expect(targetRects.last['y'], rect2.top);
    expect(targetRects.last['width'], rect2.width);
    expect(targetRects.last['height'], rect2.height);

    controller.hide();
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

    expect(targetRects, isEmpty);
    expect(hideCount, 0);
    expect(systemHideCount, 0);

    // Showing calls the platform.
    const Rect rect1 = Rect.fromLTWH(0.0, 0.0, 100.0, 100.0);
    controller.show(rect1);
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
    expect(hideCount, 0);
    expect(systemHideCount, 1);

    // Hiding does not call the platform, since the menu was already hidden.
    controller.hide();
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
    const Rect rect1 = Rect.fromLTWH(0.0, 0.0, 100.0, 100.0);
    expect(() {
      controller1.show(rect1);
    }, isNot(throwsAssertionError));

    final SystemContextMenuController controller2 = SystemContextMenuController();
    addTearDown(() {
      controller2.dispose();
    });
    const Rect rect2 = Rect.fromLTWH(1.0, 1.0, 200.0, 200.0);
    expect(() {
      controller2.show(rect2);
    }, throwsAssertionError);

    controller1.hide();
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

    expect(targetRects, isEmpty);
    expect(hideCount, 0);

    // Showing calls the platform.
    const Rect rect1 = Rect.fromLTWH(0.0, 0.0, 100.0, 100.0);
    controller1.show(rect1);
    expect(targetRects, hasLength(1));
    expect(targetRects.last['x'], rect1.left);

    // Hiding calls the platform.
    controller1.hide();
    expect(hideCount, 1);

    // Showing a new controller calls the platform.
    final SystemContextMenuController controller2 = SystemContextMenuController();
    addTearDown(() {
      controller2.dispose();
    });
    const Rect rect2 = Rect.fromLTWH(1.0, 1.0, 200.0, 200.0);
    controller2.show(rect2);
    expect(targetRects, hasLength(2));
    expect(targetRects.last['x'], rect2.left);
    expect(targetRects.last['y'], rect2.top);
    expect(targetRects.last['width'], rect2.width);
    expect(targetRects.last['height'], rect2.height);

    // Hiding the old controller does nothing.
    controller1.hide();
    expect(hideCount, 1);

    // Hiding the new controller calls the platform.
    controller2.hide();
    expect(hideCount, 2);
  });
}
