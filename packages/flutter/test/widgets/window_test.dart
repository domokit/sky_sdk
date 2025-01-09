// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

Future<Object?>? Function(MethodCall)? _createWindowMethodCallHandler(WidgetTester tester) {
  return (MethodCall call) async {
    final Map<Object?, Object?> args = call.arguments as Map<Object?, Object?>;
    if (call.method == 'createWindow') {
      final List<Object?> size = args['size']! as List<Object?>;

      await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
        SystemChannels.windowing.name,
        SystemChannels.windowing.codec.encodeMethodCall(
          MethodCall('onWindowCreated',  <String, Object?>{'viewId': tester.view.viewId, 'parentViewId': null}),
        ),
        (ByteData? data) {},
      );

      return <String, Object?>{
        'viewId': tester.view.viewId,
        'archetype': WindowArchetype.regular.index,
        'size': size,
        'parentViewId': null,
      };
    } else if (call.method == 'destroyWindow') {
      await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
        SystemChannels.windowing.name,
        SystemChannels.windowing.codec.encodeMethodCall(
          MethodCall('onWindowDestroyed',  <String, Object?>{'viewId': tester.view.viewId}),
        ),
        (ByteData? data) {},
      );

      return null;
    }

    throw Exception('Unsupported method call: ${call.method}');
  };
}

void main() {
  testWidgets('RegularWindow widget populates the controller with proper values', (
    WidgetTester tester,
  ) async {
    const Size windowSize = Size(800, 600);

    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.windowing,
      _createWindowMethodCallHandler(tester),
    );

    final RegularWindowController controller = RegularWindowController();
    await tester.pumpWidget(
      wrapWithView: false,
      Builder(
        builder: (BuildContext context) {
          return WindowingApp(
            children: <Widget>[
              RegularWindow(controller: controller, preferredSize: windowSize, child: Container()),
            ],
          );
        },
      ),
    );

    await tester.pump();

    expect(controller.type, WindowArchetype.regular);
    expect(controller.size, windowSize);
    expect(controller.view!.viewId, tester.view.viewId);
  });

  testWidgets('RegularWindow.onError is called when creation throws an error', (
    WidgetTester tester,
  ) async {
    const Size windowSize = Size(800, 600);

    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.windowing, (
      MethodCall call,
    ) async {
      throw Exception('Failed to create the window');
    });

    final RegularWindowController controller = RegularWindowController();
    bool receivedError = false;
    await tester.pumpWidget(
      wrapWithView: false,
      Builder(
        builder: (BuildContext context) {
          return WindowingApp(
            children: <Widget>[
              RegularWindow(
                controller: controller,
                onError: (String? error) {
                  expect(
                    error,
                    'PlatformException(error, Exception: Failed to create the window, null, null)',
                  );
                  receivedError = true;
                },
                preferredSize: windowSize,
                child: Container(),
              ),
            ],
          );
        },
      ),
    );

    await tester.pump();

    expect(receivedError, true);
  });

  testWidgets('RegularWindowController.destroy results in the RegularWindow.onDestroyed callback', (
    WidgetTester tester,
  ) async {
    const Size windowSize = Size(800, 600);

    tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(
      SystemChannels.windowing,
      _createWindowMethodCallHandler(tester),
    );

    bool destroyed = false;
    final RegularWindowController controller = RegularWindowController();
    await tester.pumpWidget(
      wrapWithView: false,
      Builder(
        builder: (BuildContext context) {
          return WindowingApp(
            children: <Widget>[
              RegularWindow(
                controller: controller,
                preferredSize: windowSize,
                onDestroyed: () {
                  destroyed = true;
                },
                child: Container(),
              ),
            ],
          );
        },
      ),
    );

    await tester.pump();
    await controller.destroy();

    await tester.pump();
    expect(destroyed, true);
  });

  testWidgets(
    'RegularWindowController.size is updated when an onWindowChanged event is triggered on the channel',
    (WidgetTester tester) async {
      const Size initialSize = Size(800, 600);
      const Size newSize = Size(400, 300);

      tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(
        SystemChannels.windowing,
        _createWindowMethodCallHandler(tester),
      );

      final RegularWindowController controller = RegularWindowController();
      await tester.pumpWidget(
        wrapWithView: false,
        Builder(
          builder: (BuildContext context) {
            return WindowingApp(
              children: <Widget>[
                RegularWindow(
                  controller: controller,
                  preferredSize: initialSize,
                  child: Container(),
                ),
              ],
            );
          },
        ),
      );

      await tester.pump();

      await tester.binding.defaultBinaryMessenger.handlePlatformMessage(
        SystemChannels.windowing.name,
        SystemChannels.windowing.codec.encodeMethodCall(
          MethodCall('onWindowChanged',  <String, Object?>{
            'viewId': tester.view.viewId,
            'size': <int>[newSize.width.toInt(), newSize.height.toInt()],
          }),
        ),
        (ByteData? data) {},
      );
      await tester.pump();

      expect(controller.size, newSize);
    },
  );
}
