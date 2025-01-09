// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter/src/foundation/diagnostics.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  late FakeMenuChannel fakeMenuChannel;
  late PlatformMenuDelegate originalDelegate;
  late DefaultPlatformMenuDelegate delegate;
  final List<String> selected = <String>[];
  final List<String> opened = <String>[];
  final List<String> closed = <String>[];

  void onSelected(String item) {
    selected.add(item);
  }

  void onOpen(String item) {
    opened.add(item);
  }

  void onClose(String item) {
    closed.add(item);
  }

  setUp(() {
    fakeMenuChannel = FakeMenuChannel((MethodCall call) async {});
    delegate = DefaultPlatformMenuDelegate(channel: fakeMenuChannel);
    originalDelegate = WidgetsBinding.instance.platformMenuDelegate;
    WidgetsBinding.instance.platformMenuDelegate = delegate;
    selected.clear();
    opened.clear();
    closed.clear();
  });

  tearDown(() {
    WidgetsBinding.instance.platformMenuDelegate = originalDelegate;
  });

  group('PlatformMenuBar', () {
    group('basic menu structure is transmitted to platform', () {
      testWidgets('using onSelected', (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: Material(
              child: PlatformMenuBar(
                menus: createTestMenus(
                  onSelected: onSelected,
                  onOpen: onOpen,
                  onClose: onClose,
                  shortcuts: <String, MenuSerializableShortcut>{
                    subSubMenu10[0]: const SingleActivator(LogicalKeyboardKey.keyA, control: true),
                    subSubMenu10[1]: const SingleActivator(LogicalKeyboardKey.keyB, shift: true),
                    subSubMenu10[2]: const SingleActivator(LogicalKeyboardKey.keyC, alt: true),
                    subSubMenu10[3]: const SingleActivator(LogicalKeyboardKey.keyD, meta: true),
                  },
                ),
                child: const Center(child: Text('Body')),
              ),
            ),
          ),
        );

        expect(fakeMenuChannel.outgoingCalls.last.method, equals('Menu.setMenus'));
        expect(fakeMenuChannel.outgoingCalls.last.arguments, equals(expectedStructure));
      });
      testWidgets('using onSelectedIntent', (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            home: Material(
              child: PlatformMenuBar(
                menus: createTestMenus(
                  onSelectedIntent: const DoNothingIntent(),
                  onOpen: onOpen,
                  onClose: onClose,
                  shortcuts: <String, MenuSerializableShortcut>{
                    subSubMenu10[0]: const SingleActivator(LogicalKeyboardKey.keyA, control: true),
                    subSubMenu10[1]: const SingleActivator(LogicalKeyboardKey.keyB, shift: true),
                    subSubMenu10[2]: const SingleActivator(LogicalKeyboardKey.keyC, alt: true),
                    subSubMenu10[3]: const SingleActivator(LogicalKeyboardKey.keyD, meta: true),
                  },
                ),
                child: const Center(child: Text('Body')),
              ),
            ),
          ),
        );

        expect(fakeMenuChannel.outgoingCalls.last.method, equals('Menu.setMenus'));
        expect(fakeMenuChannel.outgoingCalls.last.arguments, equals(expectedStructure));
      });
    });
    testWidgets(
      'asserts when more than one has locked the delegate',
      experimentalLeakTesting:
          LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
      (WidgetTester tester) async {
        await tester.pumpWidget(
          const MaterialApp(
            home: Material(
              child: PlatformMenuBar(
                menus: <PlatformMenuItem>[],
                child: PlatformMenuBar(menus: <PlatformMenuItem>[], child: SizedBox()),
              ),
            ),
          ),
        );
        expect(tester.takeException(), isA<AssertionError>());
      },
    );
    testWidgets('diagnostics', (WidgetTester tester) async {
      const PlatformMenuItem item = PlatformMenuItem(
        label: 'label2',
        shortcut: SingleActivator(LogicalKeyboardKey.keyA),
      );
      const PlatformMenuBar menuBar = PlatformMenuBar(
        menus: <PlatformMenuItem>[item],
        child: SizedBox(),
      );

      await tester.pumpWidget(const MaterialApp(home: Material(child: menuBar)));
      await tester.pump();

      expect(
        menuBar.toStringDeep(),
        equalsIgnoringHashCodes(
          'PlatformMenuBar#00000\n'
          ' └─PlatformMenuItem#00000(label2)\n'
          '     label: "label2"\n'
          '     shortcut: SingleActivator#00000(keys: Key A)\n'
          '     DISABLED\n',
        ),
      );
    });
  });
  group('MenuBarItem', () {
    testWidgets('diagnostics', (WidgetTester tester) async {
      const PlatformMenuItem childItem = PlatformMenuItem(label: 'label');
      const PlatformMenu item = PlatformMenu(label: 'label', menus: <PlatformMenuItem>[childItem]);

      final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
      item.debugFillProperties(builder);

      final List<String> description =
          builder.properties
              .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
              .map((DiagnosticsNode node) => node.toString())
              .toList();

      expect(description, <String>['label: "label"']);
    });
  });

  group('ShortcutSerialization', () {
    testWidgets('character constructor', (WidgetTester tester) async {
      final ShortcutSerialization serialization = ShortcutSerialization.character('?');
      expect(
        serialization.toChannelRepresentation(),
        equals(<String, Object?>{'shortcutCharacter': '?', 'shortcutModifiers': 0}),
      );
      final ShortcutSerialization serializationWithModifiers = ShortcutSerialization.character(
        '?',
        alt: true,
        control: true,
        meta: true,
      );
      expect(
        serializationWithModifiers.toChannelRepresentation(),
        equals(<String, Object?>{'shortcutCharacter': '?', 'shortcutModifiers': 13}),
      );
    });

    testWidgets('modifier constructor', (WidgetTester tester) async {
      final ShortcutSerialization serialization = ShortcutSerialization.modifier(
        LogicalKeyboardKey.home,
      );
      expect(
        serialization.toChannelRepresentation(),
        equals(<String, Object?>{
          'shortcutTrigger': LogicalKeyboardKey.home.keyId,
          'shortcutModifiers': 0,
        }),
      );
      final ShortcutSerialization serializationWithModifiers = ShortcutSerialization.modifier(
        LogicalKeyboardKey.home,
        alt: true,
        control: true,
        meta: true,
        shift: true,
      );
      expect(
        serializationWithModifiers.toChannelRepresentation(),
        equals(<String, Object?>{
          'shortcutTrigger': LogicalKeyboardKey.home.keyId,
          'shortcutModifiers': 15,
        }),
      );
    });
  });
}

const List<String> mainMenu = <String>['Menu 0', 'Menu 1', 'Menu 2', 'Menu 3'];

const List<String> subMenu0 = <String>['Sub Menu 00'];

const List<String> subMenu1 = <String>['Sub Menu 10', 'Sub Menu 11', 'Sub Menu 12'];

const List<String> subSubMenu10 = <String>[
  'Sub Sub Menu 110',
  'Sub Sub Menu 111',
  'Sub Sub Menu 112',
  'Sub Sub Menu 113',
];

const List<String> subMenu2 = <String>['Sub Menu 20'];

List<PlatformMenuItem> createTestMenus({
  void Function(String)? onSelected,
  Intent? onSelectedIntent,
  void Function(String)? onOpen,
  void Function(String)? onClose,
  Map<String, MenuSerializableShortcut> shortcuts = const <String, MenuSerializableShortcut>{},
  bool includeStandard = false,
}) {
  final List<PlatformMenuItem> result = <PlatformMenuItem>[
    PlatformMenu(
      label: mainMenu[0],
      onOpen: onOpen != null ? () => onOpen(mainMenu[0]) : null,
      onClose: onClose != null ? () => onClose(mainMenu[0]) : null,
      menus: <PlatformMenuItem>[
        PlatformMenuItem(
          label: subMenu0[0],
          onSelected: onSelected != null ? () => onSelected(subMenu0[0]) : null,
          onSelectedIntent: onSelectedIntent,
          shortcut: shortcuts[subMenu0[0]],
        ),
      ],
    ),
    PlatformMenu(
      label: mainMenu[1],
      onOpen: onOpen != null ? () => onOpen(mainMenu[1]) : null,
      onClose: onClose != null ? () => onClose(mainMenu[1]) : null,
      menus: <PlatformMenuItem>[
        PlatformMenuItemGroup(
          members: <PlatformMenuItem>[
            PlatformMenuItem(
              label: subMenu1[0],
              onSelected: onSelected != null ? () => onSelected(subMenu0[0]) : null,
              onSelectedIntent: onSelectedIntent,
              shortcut: shortcuts[subMenu1[0]],
            ),
          ],
        ),
        PlatformMenu(
          label: subMenu1[1],
          onOpen: onOpen != null ? () => onOpen(subMenu1[1]) : null,
          onClose: onClose != null ? () => onClose(subMenu1[1]) : null,
          menus: <PlatformMenuItem>[
            PlatformMenuItemGroup(
              members: <PlatformMenuItem>[
                PlatformMenuItem(
                  label: subSubMenu10[0],
                  onSelected: onSelected != null ? () => onSelected(subSubMenu10[0]) : null,
                  onSelectedIntent: onSelectedIntent,
                  shortcut: shortcuts[subSubMenu10[0]],
                ),
              ],
            ),
            PlatformMenuItemGroup(
              members: <PlatformMenuItem>[
                PlatformMenuItem(
                  label: subSubMenu10[1],
                  onSelected: onSelected != null ? () => onSelected(subSubMenu10[1]) : null,
                  onSelectedIntent: onSelectedIntent,
                  shortcut: shortcuts[subSubMenu10[1]],
                ),
              ],
            ),
            PlatformMenuItem(
              label: subSubMenu10[2],
              onSelected: onSelected != null ? () => onSelected(subSubMenu10[2]) : null,
              onSelectedIntent: onSelectedIntent,
              shortcut: shortcuts[subSubMenu10[2]],
            ),
            PlatformMenuItemGroup(
              members: <PlatformMenuItem>[
                PlatformMenuItem(
                  label: subSubMenu10[3],
                  onSelected: onSelected != null ? () => onSelected(subSubMenu10[3]) : null,
                  onSelectedIntent: onSelectedIntent,
                  shortcut: shortcuts[subSubMenu10[3]],
                ),
              ],
            ),
          ],
        ),
        PlatformMenuItem(
          label: subMenu1[2],
          onSelected: onSelected != null ? () => onSelected(subMenu1[2]) : null,
          onSelectedIntent: onSelectedIntent,
          shortcut: shortcuts[subMenu1[2]],
        ),
      ],
    ),
    PlatformMenu(
      label: mainMenu[2],
      onOpen: onOpen != null ? () => onOpen(mainMenu[2]) : null,
      onClose: onClose != null ? () => onClose(mainMenu[2]) : null,
      menus: <PlatformMenuItem>[
        PlatformMenuItem(
          // Always disabled.
          label: subMenu2[0],
          shortcut: shortcuts[subMenu2[0]],
        ),
      ],
    ),
    // Disabled menu
    PlatformMenu(
      label: mainMenu[3],
      onOpen: onOpen != null ? () => onOpen(mainMenu[2]) : null,
      onClose: onClose != null ? () => onClose(mainMenu[2]) : null,
      menus: <PlatformMenuItem>[],
    ),
  ];
  return result;
}

const Map<String, Object?> expectedStructure = <String, Object?>{
  '0': <Map<String, Object?>>[
    <String, Object?>{
      'id': 2,
      'label': 'Menu 0',
      'enabled': true,
      'children': <Map<String, Object?>>[
        <String, Object?>{'id': 1, 'label': 'Sub Menu 00', 'enabled': true},
      ],
    },
    <String, Object?>{
      'id': 18,
      'label': 'Menu 1',
      'enabled': true,
      'children': <Map<String, Object?>>[
        <String, Object?>{'id': 4, 'label': 'Sub Menu 10', 'enabled': true},
        <String, Object?>{'id': 5, 'isDivider': true},
        <String, Object?>{
          'id': 16,
          'label': 'Sub Menu 11',
          'enabled': true,
          'children': <Map<String, Object?>>[
            <String, Object?>{
              'id': 7,
              'label': 'Sub Sub Menu 110',
              'enabled': true,
              'shortcutTrigger': 97,
              'shortcutModifiers': 8,
            },
            <String, Object?>{'id': 8, 'isDivider': true},
            <String, Object?>{
              'id': 10,
              'label': 'Sub Sub Menu 111',
              'enabled': true,
              'shortcutTrigger': 98,
              'shortcutModifiers': 2,
            },
            <String, Object?>{'id': 11, 'isDivider': true},
            <String, Object?>{
              'id': 12,
              'label': 'Sub Sub Menu 112',
              'enabled': true,
              'shortcutTrigger': 99,
              'shortcutModifiers': 4,
            },
            <String, Object?>{'id': 13, 'isDivider': true},
            <String, Object?>{
              'id': 14,
              'label': 'Sub Sub Menu 113',
              'enabled': true,
              'shortcutTrigger': 100,
              'shortcutModifiers': 1,
            },
          ],
        },
        <String, Object?>{'id': 17, 'label': 'Sub Menu 12', 'enabled': true},
      ],
    },
    <String, Object?>{
      'id': 20,
      'label': 'Menu 2',
      'enabled': true,
      'children': <Map<String, Object?>>[
        <String, Object?>{'id': 19, 'label': 'Sub Menu 20', 'enabled': false},
      ],
    },
    <String, Object?>{
      'id': 21,
      'label': 'Menu 3',
      'enabled': false,
      'children': <Map<String, Object?>>[],
    },
  ],
};

class FakeMenuChannel implements MethodChannel {
  FakeMenuChannel(this.outgoing);

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
