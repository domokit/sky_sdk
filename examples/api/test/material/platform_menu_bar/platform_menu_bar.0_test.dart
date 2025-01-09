// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/material/platform_menu_bar/platform_menu_bar.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  late _FakeMenuChannel fakeMenuChannel;
  late PlatformMenuDelegate originalDelegate;
  late DefaultPlatformMenuDelegate delegate;

  setUp(() {
    fakeMenuChannel = _FakeMenuChannel((MethodCall call) async {});
    delegate = DefaultPlatformMenuDelegate(channel: fakeMenuChannel);
    originalDelegate = WidgetsBinding.instance.platformMenuDelegate;
    WidgetsBinding.instance.platformMenuDelegate = delegate;
  });

  tearDown(() {
    WidgetsBinding.instance.platformMenuDelegate = originalDelegate;
  });

  testWidgets(
    'PlatformMenuBar creates a menu',
    (WidgetTester tester) async {
      await tester.pumpWidget(const example.ExampleApp());

      expect(
        find.text('This space intentionally left blank.\nShow a message here using the menu.'),
        findsOne,
      );
      expect(find.byType(PlatformMenuBar), findsOne);

      expect(fakeMenuChannel.outgoingCalls.last.method, 'Menu.setMenus');
      expect(
        fakeMenuChannel.outgoingCalls.last.arguments,
        equals(const <String, Object?>{
          '0': <Map<String, Object>>[
            <String, Object>{
              'id': 11,
              'label': 'Flutter API Sample',
              'enabled': true,
              'children': <Map<String, Object>>[
                <String, Object>{'id': 2, 'label': 'About', 'enabled': true},
                <String, Object>{'id': 3, 'isDivider': true},
                <String, Object>{
                  'id': 5,
                  'label': 'Show Message',
                  'enabled': true,
                  'shortcutCharacter': 'm',
                  'shortcutModifiers': 0,
                },
                <String, Object>{
                  'id': 8,
                  'label': 'Messages',
                  'enabled': true,
                  'children': <Map<String, Object>>[
                    <String, Object>{
                      'id': 6,
                      'label': 'I am not throwing away my shot.',
                      'enabled': true,
                      'shortcutTrigger': 49,
                      'shortcutModifiers': 1,
                    },
                    <String, Object>{
                      'id': 7,
                      'label': "There's a million things I haven't done, but just you wait.",
                      'enabled': true,
                      'shortcutTrigger': 50,
                      'shortcutModifiers': 1,
                    },
                  ],
                },
                <String, Object>{'id': 9, 'isDivider': true},
                <String, Object>{'id': 10, 'enabled': true, 'platformProvidedMenu': 1},
              ],
            },
          ],
        }),
      );
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{TargetPlatform.macOS}),
  );
}

class _FakeMenuChannel implements MethodChannel {
  _FakeMenuChannel(this.outgoing);

  Future<dynamic> Function(MethodCall) outgoing;
  Future<void> Function(MethodCall)? incoming;

  List<MethodCall> outgoingCalls = <MethodCall>[];

  @override
  BinaryMessenger get binaryMessenger => throw UnimplementedError();

  @override
  MethodCodec get codec => const StandardMethodCodec();

  @override
  Future<List<T>> invokeListMethod<T>(String method, [dynamic arguments]) =>
      throw UnimplementedError();

  @override
  Future<Map<K, V>> invokeMapMethod<K, V>(String method, [dynamic arguments]) =>
      throw UnimplementedError();

  @override
  Future<T> invokeMethod<T>(String method, [dynamic arguments]) async {
    final MethodCall call = MethodCall(method, arguments);
    outgoingCalls.add(call);
    return await outgoing(call) as T;
  }

  @override
  String get name => 'flutter/menu';

  @override
  void setMethodCallHandler(Future<void> Function(MethodCall call)? handler) => incoming = handler;
}
