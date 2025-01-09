// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/feedback_tester.dart';
import '../widgets/semantics_tester.dart';

Widget wrap({Widget? child}) {
  return MediaQuery(
    data: const MediaQueryData(),
    child: Directionality(textDirection: TextDirection.ltr, child: Material(child: child)),
  );
}

void main() {
  testWidgets('RadioListTile should initialize according to groupValue', (
    WidgetTester tester,
  ) async {
    final List<int> values = <int>[0, 1, 2];
    int? selectedValue;
    // Constructor parameters are required for [RadioListTile], but they are
    // irrelevant when searching with [find.byType].
    final Type radioListTileType =
        const RadioListTile<int>(value: 0, groupValue: 0, onChanged: null).runtimeType;

    List<RadioListTile<int>> generatedRadioListTiles;
    List<RadioListTile<int>> findTiles() =>
        find
            .byType(radioListTileType)
            .evaluate()
            .map<Widget>((Element element) => element.widget)
            .cast<RadioListTile<int>>()
            .toList();

    Widget buildFrame() {
      return wrap(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Scaffold(
              body: ListView.builder(
                itemCount: values.length,
                itemBuilder:
                    (BuildContext context, int index) => RadioListTile<int>(
                      onChanged: (int? value) {
                        setState(() {
                          selectedValue = value;
                        });
                      },
                      value: values[index],
                      groupValue: selectedValue,
                      title: Text(values[index].toString()),
                    ),
              ),
            );
          },
        ),
      );
    }

    await tester.pumpWidget(buildFrame());
    generatedRadioListTiles = findTiles();

    expect(generatedRadioListTiles[0].checked, equals(false));
    expect(generatedRadioListTiles[1].checked, equals(false));
    expect(generatedRadioListTiles[2].checked, equals(false));

    selectedValue = 1;

    await tester.pumpWidget(buildFrame());
    generatedRadioListTiles = findTiles();

    expect(generatedRadioListTiles[0].checked, equals(false));
    expect(generatedRadioListTiles[1].checked, equals(true));
    expect(generatedRadioListTiles[2].checked, equals(false));
  });

  testWidgets('RadioListTile simple control test', (WidgetTester tester) async {
    final Key key = UniqueKey();
    final Key titleKey = UniqueKey();
    final List<int?> log = <int?>[];

    await tester.pumpWidget(
      wrap(
        child: RadioListTile<int>(
          key: key,
          value: 1,
          groupValue: 2,
          onChanged: log.add,
          title: Text('Title', key: titleKey),
        ),
      ),
    );

    await tester.tap(find.byKey(key));

    expect(log, equals(<int>[1]));
    log.clear();

    await tester.pumpWidget(
      wrap(
        child: RadioListTile<int>(
          key: key,
          value: 1,
          groupValue: 1,
          onChanged: log.add,
          activeColor: Colors.green[500],
          title: Text('Title', key: titleKey),
        ),
      ),
    );

    await tester.tap(find.byKey(key));

    expect(log, isEmpty);

    await tester.pumpWidget(
      wrap(
        child: RadioListTile<int>(
          key: key,
          value: 1,
          groupValue: 2,
          onChanged: null,
          title: Text('Title', key: titleKey),
        ),
      ),
    );

    await tester.tap(find.byKey(key));

    expect(log, isEmpty);

    await tester.pumpWidget(
      wrap(
        child: RadioListTile<int>(
          key: key,
          value: 1,
          groupValue: 2,
          onChanged: log.add,
          title: Text('Title', key: titleKey),
        ),
      ),
    );

    await tester.tap(find.byKey(titleKey));

    expect(log, equals(<int>[1]));
  });

  testWidgets('RadioListTile control tests', (WidgetTester tester) async {
    final List<int> values = <int>[0, 1, 2];
    int? selectedValue;
    // Constructor parameters are required for [Radio], but they are irrelevant
    // when searching with [find.byType].
    final Type radioType = const Radio<int>(value: 0, groupValue: 0, onChanged: null).runtimeType;
    final List<dynamic> log = <dynamic>[];

    Widget buildFrame() {
      return wrap(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Scaffold(
              body: ListView.builder(
                itemCount: values.length,
                itemBuilder:
                    (BuildContext context, int index) => RadioListTile<int>(
                      onChanged: (int? value) {
                        log.add(value);
                        setState(() {
                          selectedValue = value;
                        });
                      },
                      value: values[index],
                      groupValue: selectedValue,
                      title: Text(values[index].toString()),
                    ),
              ),
            );
          },
        ),
      );
    }

    // Tests for tapping between [Radio] and [ListTile]
    await tester.pumpWidget(buildFrame());
    await tester.tap(find.text('1'));
    log.add('-');
    await tester.tap(find.byType(radioType).at(2));
    expect(log, equals(<dynamic>[1, '-', 2]));
    log.add('-');
    await tester.tap(find.text('1'));

    log.clear();
    selectedValue = null;

    // Tests for tapping across [Radio]s exclusively
    await tester.pumpWidget(buildFrame());
    await tester.tap(find.byType(radioType).at(1));
    log.add('-');
    await tester.tap(find.byType(radioType).at(2));
    expect(log, equals(<dynamic>[1, '-', 2]));

    log.clear();
    selectedValue = null;

    // Tests for tapping across [ListTile]s exclusively
    await tester.pumpWidget(buildFrame());
    await tester.tap(find.text('1'));
    log.add('-');
    await tester.tap(find.text('2'));
    expect(log, equals(<dynamic>[1, '-', 2]));
  });

  testWidgets('Selected RadioListTile should not trigger onChanged', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/30311
    final List<int> values = <int>[0, 1, 2];
    int? selectedValue;
    // Constructor parameters are required for [Radio], but they are irrelevant
    // when searching with [find.byType].
    final Type radioType = const Radio<int>(value: 0, groupValue: 0, onChanged: null).runtimeType;
    final List<dynamic> log = <dynamic>[];

    Widget buildFrame() {
      return wrap(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Scaffold(
              body: ListView.builder(
                itemCount: values.length,
                itemBuilder:
                    (BuildContext context, int index) => RadioListTile<int>(
                      onChanged: (int? value) {
                        log.add(value);
                        setState(() {
                          selectedValue = value;
                        });
                      },
                      value: values[index],
                      groupValue: selectedValue,
                      title: Text(values[index].toString()),
                    ),
              ),
            );
          },
        ),
      );
    }

    await tester.pumpWidget(buildFrame());
    await tester.tap(find.text('0'));
    await tester.pump();
    expect(log, equals(<int>[0]));

    await tester.tap(find.text('0'));
    await tester.pump();
    expect(log, equals(<int>[0]));

    await tester.tap(find.byType(radioType).at(0));
    await tester.pump();
    expect(log, equals(<int>[0]));
  });

  testWidgets('Selected RadioListTile should trigger onChanged when toggleable', (
    WidgetTester tester,
  ) async {
    final List<int> values = <int>[0, 1, 2];
    int? selectedValue;
    // Constructor parameters are required for [Radio], but they are irrelevant
    // when searching with [find.byType].
    final Type radioType = const Radio<int>(value: 0, groupValue: 0, onChanged: null).runtimeType;
    final List<dynamic> log = <dynamic>[];

    Widget buildFrame() {
      return wrap(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Scaffold(
              body: ListView.builder(
                itemCount: values.length,
                itemBuilder: (BuildContext context, int index) {
                  return RadioListTile<int>(
                    onChanged: (int? value) {
                      log.add(value);
                      setState(() {
                        selectedValue = value;
                      });
                    },
                    toggleable: true,
                    value: values[index],
                    groupValue: selectedValue,
                    title: Text(values[index].toString()),
                  );
                },
              ),
            );
          },
        ),
      );
    }

    await tester.pumpWidget(buildFrame());
    await tester.tap(find.text('0'));
    await tester.pump();
    expect(log, equals(<int>[0]));

    await tester.tap(find.text('0'));
    await tester.pump();
    expect(log, equals(<int?>[0, null]));

    await tester.tap(find.byType(radioType).at(0));
    await tester.pump();
    expect(log, equals(<int?>[0, null, 0]));
  });

  testWidgets('RadioListTile can be toggled when toggleable is set', (WidgetTester tester) async {
    final Key key = UniqueKey();
    final List<int?> log = <int?>[];

    await tester.pumpWidget(
      Material(
        child: Center(
          child: Radio<int>(
            key: key,
            value: 1,
            groupValue: 2,
            onChanged: log.add,
            toggleable: true,
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(key));

    expect(log, equals(<int>[1]));
    log.clear();

    await tester.pumpWidget(
      Material(
        child: Center(
          child: Radio<int>(
            key: key,
            value: 1,
            groupValue: 1,
            onChanged: log.add,
            toggleable: true,
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(key));

    expect(log, equals(<int?>[null]));
    log.clear();

    await tester.pumpWidget(
      Material(
        child: Center(
          child: Radio<int>(
            key: key,
            value: 1,
            groupValue: null,
            onChanged: log.add,
            toggleable: true,
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(key));

    expect(log, equals(<int>[1]));
  });

  testWidgets('RadioListTile semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      wrap(
        child: RadioListTile<int>(
          value: 1,
          groupValue: 2,
          onChanged: (int? i) {},
          title: const Text('Title'),
          internalAddSemanticForOnTap: true,
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: 1,
              flags: <SemanticsFlag>[
                SemanticsFlag.isButton,
                SemanticsFlag.hasCheckedState,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isInMutuallyExclusiveGroup,
                SemanticsFlag.isFocusable,
                SemanticsFlag.hasSelectedState,
              ],
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
              label: 'Title',
              textDirection: TextDirection.ltr,
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    await tester.pumpWidget(
      wrap(
        child: RadioListTile<int>(
          value: 2,
          groupValue: 2,
          onChanged: (int? i) {},
          title: const Text('Title'),
          internalAddSemanticForOnTap: true,
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: 1,
              flags: <SemanticsFlag>[
                SemanticsFlag.isButton,
                SemanticsFlag.hasCheckedState,
                SemanticsFlag.isChecked,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isInMutuallyExclusiveGroup,
                SemanticsFlag.isFocusable,
                SemanticsFlag.hasSelectedState,
              ],
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
              label: 'Title',
              textDirection: TextDirection.ltr,
            ),
          ],
        ),
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    await tester.pumpWidget(
      wrap(
        child: const RadioListTile<int>(
          value: 1,
          groupValue: 2,
          onChanged: null,
          title: Text('Title'),
          internalAddSemanticForOnTap: true,
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: 1,
              flags: <SemanticsFlag>[
                SemanticsFlag.hasCheckedState,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isInMutuallyExclusiveGroup,
                SemanticsFlag.isFocusable,
                SemanticsFlag.hasSelectedState,
              ],
              actions: <SemanticsAction>[SemanticsAction.focus],
              label: 'Title',
              textDirection: TextDirection.ltr,
            ),
          ],
        ),
        ignoreId: true,
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    await tester.pumpWidget(
      wrap(
        child: const RadioListTile<int>(
          value: 2,
          groupValue: 2,
          onChanged: null,
          title: Text('Title'),
          internalAddSemanticForOnTap: true,
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              id: 1,
              flags: <SemanticsFlag>[
                SemanticsFlag.hasCheckedState,
                SemanticsFlag.isChecked,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isInMutuallyExclusiveGroup,
                SemanticsFlag.hasSelectedState,
              ],
              label: 'Title',
              textDirection: TextDirection.ltr,
            ),
          ],
        ),
        ignoreId: true,
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('RadioListTile has semantic events', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    final Key key = UniqueKey();
    dynamic semanticEvent;
    int? radioValue = 2;
    tester.binding.defaultBinaryMessenger.setMockDecodedMessageHandler<dynamic>(
      SystemChannels.accessibility,
      (dynamic message) async {
        semanticEvent = message;
      },
    );

    await tester.pumpWidget(
      wrap(
        child: RadioListTile<int>(
          key: key,
          value: 1,
          groupValue: radioValue,
          onChanged: (int? i) {
            radioValue = i;
          },
          title: const Text('Title'),
        ),
      ),
    );

    await tester.tap(find.byKey(key));
    await tester.pump();
    final RenderObject object = tester.firstRenderObject(find.byKey(key));

    expect(radioValue, 1);
    expect(semanticEvent, <String, dynamic>{
      'type': 'tap',
      'nodeId': object.debugSemantics!.id,
      'data': <String, dynamic>{},
    });
    expect(object.debugSemantics!.getSemanticsData().hasAction(SemanticsAction.tap), true);

    semantics.dispose();
    tester.binding.defaultBinaryMessenger.setMockDecodedMessageHandler<dynamic>(
      SystemChannels.accessibility,
      null,
    );
  });

  testWidgets('RadioListTile can autofocus unless disabled.', (WidgetTester tester) async {
    final GlobalKey childKey = GlobalKey();

    await tester.pumpWidget(
      wrap(
        child: RadioListTile<int>(
          value: 1,
          groupValue: 2,
          onChanged: (_) {},
          title: Text('Title', key: childKey),
          autofocus: true,
        ),
      ),
    );

    await tester.pump();
    expect(Focus.of(childKey.currentContext!).hasPrimaryFocus, isTrue);

    await tester.pumpWidget(
      wrap(
        child: RadioListTile<int>(
          value: 1,
          groupValue: 2,
          onChanged: null,
          title: Text('Title', key: childKey),
          autofocus: true,
        ),
      ),
    );

    await tester.pump();
    expect(Focus.of(childKey.currentContext!).hasPrimaryFocus, isFalse);
  });

  testWidgets('RadioListTile contentPadding test', (WidgetTester tester) async {
    final Type radioType =
        const Radio<bool>(groupValue: true, value: true, onChanged: null).runtimeType;

    await tester.pumpWidget(
      wrap(
        child: Center(
          child: RadioListTile<bool>(
            groupValue: true,
            value: true,
            title: const Text('Title'),
            onChanged: (_) {},
            contentPadding: const EdgeInsets.fromLTRB(8, 10, 15, 20),
          ),
        ),
      ),
    );

    final Rect paddingRect = tester.getRect(find.byType(SafeArea));
    final Rect radioRect = tester.getRect(find.byType(radioType));
    final Rect titleRect = tester.getRect(find.text('Title'));

    // Get the taller Rect of the Radio and Text widgets
    final Rect tallerRect = radioRect.height > titleRect.height ? radioRect : titleRect;

    // Get the extra height between the tallerRect and ListTile height
    final double extraHeight = 56 - tallerRect.height;

    // Check for correct top and bottom padding
    expect(paddingRect.top, tallerRect.top - extraHeight / 2 - 10); //top padding
    expect(paddingRect.bottom, tallerRect.bottom + extraHeight / 2 + 20); //bottom padding

    // Check for correct left and right padding
    expect(paddingRect.left, radioRect.left - 8); //left padding
    expect(paddingRect.right, titleRect.right + 15); //right padding
  });

  testWidgets('RadioListTile respects shape', (WidgetTester tester) async {
    const ShapeBorder shapeBorder = RoundedRectangleBorder(
      borderRadius: BorderRadius.horizontal(right: Radius.circular(100)),
    );

    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: RadioListTile<bool>(
            value: true,
            groupValue: true,
            onChanged: null,
            title: Text('Title'),
            shape: shapeBorder,
          ),
        ),
      ),
    );

    expect(tester.widget<InkWell>(find.byType(InkWell)).customBorder, shapeBorder);
  });

  testWidgets('RadioListTile respects tileColor', (WidgetTester tester) async {
    final Color tileColor = Colors.red.shade500;

    await tester.pumpWidget(
      wrap(
        child: Center(
          child: RadioListTile<bool>(
            value: false,
            groupValue: true,
            onChanged: null,
            title: const Text('Title'),
            tileColor: tileColor,
          ),
        ),
      ),
    );

    expect(find.byType(Material), paints..rect(color: tileColor));
  });

  testWidgets('RadioListTile respects selectedTileColor', (WidgetTester tester) async {
    final Color selectedTileColor = Colors.green.shade500;

    await tester.pumpWidget(
      wrap(
        child: Center(
          child: RadioListTile<bool>(
            value: false,
            groupValue: true,
            onChanged: null,
            title: const Text('Title'),
            selected: true,
            selectedTileColor: selectedTileColor,
          ),
        ),
      ),
    );

    expect(find.byType(Material), paints..rect(color: selectedTileColor));
  });

  testWidgets('RadioListTile selected item text Color', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/pull/76906

    const Color activeColor = Color(0xff00ff00);

    Widget buildFrame({Color? activeColor, Color? fillColor}) {
      return MaterialApp(
        theme: ThemeData.light().copyWith(
          radioTheme: RadioThemeData(
            fillColor: MaterialStateProperty.resolveWith<Color?>((Set<MaterialState> states) {
              return states.contains(MaterialState.selected) ? fillColor : null;
            }),
          ),
        ),
        home: Scaffold(
          body: Center(
            child: RadioListTile<bool>(
              activeColor: activeColor,
              selected: true,
              title: const Text('title'),
              value: false,
              groupValue: true,
              onChanged: (bool? newValue) {},
            ),
          ),
        ),
      );
    }

    Color? textColor(String text) {
      return tester.renderObject<RenderParagraph>(find.text(text)).text.style?.color;
    }

    await tester.pumpWidget(buildFrame(fillColor: activeColor));
    expect(textColor('title'), activeColor);

    await tester.pumpWidget(buildFrame(activeColor: activeColor));
    expect(textColor('title'), activeColor);
  });

  testWidgets('RadioListTile respects visualDensity', (WidgetTester tester) async {
    const Key key = Key('test');
    Future<void> buildTest(VisualDensity visualDensity) async {
      return tester.pumpWidget(
        wrap(
          child: Center(
            child: RadioListTile<bool>(
              key: key,
              value: false,
              groupValue: true,
              onChanged: (bool? value) {},
              autofocus: true,
              visualDensity: visualDensity,
            ),
          ),
        ),
      );
    }

    await buildTest(VisualDensity.standard);
    final RenderBox box = tester.renderObject(find.byKey(key));
    await tester.pumpAndSettle();
    expect(box.size, equals(const Size(800, 56)));
  });

  testWidgets('RadioListTile respects focusNode', (WidgetTester tester) async {
    final GlobalKey childKey = GlobalKey();
    await tester.pumpWidget(
      wrap(
        child: Center(
          child: RadioListTile<bool>(
            value: false,
            groupValue: true,
            title: Text('A', key: childKey),
            onChanged: (bool? value) {},
          ),
        ),
      ),
    );

    await tester.pump();
    final FocusNode tileNode = Focus.of(childKey.currentContext!);
    tileNode.requestFocus();
    await tester.pump(); // Let the focus take effect.
    expect(Focus.of(childKey.currentContext!).hasPrimaryFocus, isTrue);
    expect(tileNode.hasPrimaryFocus, isTrue);
  });

  testWidgets('RadioListTile onFocusChange callback', (WidgetTester tester) async {
    final FocusNode node = FocusNode(debugLabel: 'RadioListTile onFocusChange');
    addTearDown(node.dispose);

    bool gotFocus = false;
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: RadioListTile<bool>(
            value: true,
            focusNode: node,
            onFocusChange: (bool focused) {
              gotFocus = focused;
            },
            onChanged: (bool? value) {},
            groupValue: true,
          ),
        ),
      ),
    );

    node.requestFocus();
    await tester.pump();
    expect(gotFocus, isTrue);
    expect(node.hasFocus, isTrue);

    node.unfocus();
    await tester.pump();
    expect(gotFocus, isFalse);
    expect(node.hasFocus, isFalse);
  });

  testWidgets('Radio changes mouse cursor when hovered', (WidgetTester tester) async {
    // Test Radio() constructor
    await tester.pumpWidget(
      wrap(
        child: MouseRegion(
          cursor: SystemMouseCursors.forbidden,
          child: RadioListTile<int>(
            mouseCursor: SystemMouseCursors.text,
            value: 1,
            onChanged: (int? v) {},
            groupValue: 2,
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.byType(Radio<int>)));

    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );

    // Test default cursor
    await tester.pumpWidget(
      wrap(
        child: MouseRegion(
          cursor: SystemMouseCursors.forbidden,
          child: RadioListTile<int>(value: 1, onChanged: (int? v) {}, groupValue: 2),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );

    // Test default cursor when disabled
    await tester.pumpWidget(
      wrap(
        child: const MouseRegion(
          cursor: SystemMouseCursors.forbidden,
          child: RadioListTile<int>(value: 1, onChanged: null, groupValue: 2),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );
  });

  testWidgets('RadioListTile respects fillColor in enabled/disabled states', (
    WidgetTester tester,
  ) async {
    const Color activeEnabledFillColor = Color(0xFF000001);
    const Color activeDisabledFillColor = Color(0xFF000002);
    const Color inactiveEnabledFillColor = Color(0xFF000003);
    const Color inactiveDisabledFillColor = Color(0xFF000004);

    Color getFillColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        if (states.contains(MaterialState.selected)) {
          return activeDisabledFillColor;
        }
        return inactiveDisabledFillColor;
      }
      if (states.contains(MaterialState.selected)) {
        return activeEnabledFillColor;
      }
      return inactiveEnabledFillColor;
    }

    final MaterialStateProperty<Color> fillColor = MaterialStateColor.resolveWith(getFillColor);

    int? groupValue = 0;
    Widget buildApp({required bool enabled}) {
      return wrap(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return RadioListTile<int>(
              value: 0,
              fillColor: fillColor,
              onChanged:
                  enabled
                      ? (int? newValue) {
                        setState(() {
                          groupValue = newValue;
                        });
                      }
                      : null,
              groupValue: groupValue,
            );
          },
        ),
      );
    }

    await tester.pumpWidget(buildApp(enabled: true));

    // Selected and enabled.
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Radio<int>))),
      paints
        ..rect()
        ..circle(color: activeEnabledFillColor)
        ..circle(color: activeEnabledFillColor),
    );

    // Check when the radio isn't selected.
    groupValue = 1;
    await tester.pumpWidget(buildApp(enabled: true));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Radio<int>))),
      paints
        ..rect()
        ..circle(color: inactiveEnabledFillColor, style: PaintingStyle.stroke, strokeWidth: 2.0),
    );

    // Check when the radio is selected, but disabled.
    groupValue = 0;
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Radio<int>))),
      paints
        ..rect()
        ..circle(color: activeDisabledFillColor)
        ..circle(color: activeDisabledFillColor),
    );

    // Check when the radio is unselected and disabled.
    groupValue = 1;
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Radio<int>))),
      paints
        ..rect()
        ..circle(color: inactiveDisabledFillColor, style: PaintingStyle.stroke, strokeWidth: 2.0),
    );
  });

  testWidgets('RadioListTile respects fillColor in hovered state', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    const Color hoveredFillColor = Color(0xFF000001);

    Color getFillColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.hovered)) {
        return hoveredFillColor;
      }
      return Colors.transparent;
    }

    final MaterialStateProperty<Color> fillColor = MaterialStateColor.resolveWith(getFillColor);

    int? groupValue = 0;
    Widget buildApp() {
      return wrap(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return RadioListTile<int>(
              value: 0,
              fillColor: fillColor,
              onChanged: (int? newValue) {
                setState(() {
                  groupValue = newValue;
                });
              },
              groupValue: groupValue,
            );
          },
        ),
      );
    }

    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byType(Radio<int>)));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Radio<int>))),
      paints
        ..rect()
        ..circle()
        ..circle(color: hoveredFillColor),
    );
  });

  testWidgets('Material3 - RadioListTile respects hoverColor', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    int? groupValue = 0;
    final Color? hoverColor = Colors.orange[500];
    final ThemeData theme = ThemeData(useMaterial3: true);
    Widget buildApp({bool enabled = true}) {
      return wrap(
        child: MaterialApp(
          theme: theme,
          home: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return RadioListTile<int>(
                value: 0,
                onChanged:
                    enabled
                        ? (int? newValue) {
                          setState(() {
                            groupValue = newValue;
                          });
                        }
                        : null,
                hoverColor: hoverColor,
                groupValue: groupValue,
              );
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp());

    await tester.pump();
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Radio<int>))),
      paints
        ..rect()
        ..circle(color: theme.colorScheme.primary)
        ..circle(color: theme.colorScheme.primary),
    );

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.moveTo(tester.getCenter(find.byType(Radio<int>)));

    // Check when the radio isn't selected.
    groupValue = 1;
    await tester.pumpWidget(buildApp());
    await tester.pump();
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Radio<int>))),
      paints
        ..rect()
        ..circle(color: hoverColor),
    );

    // Check when the radio is selected, but disabled.
    groupValue = 0;
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pump();
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Radio<int>))),
      paints
        ..rect()
        ..circle(color: theme.colorScheme.onSurface.withOpacity(0.38))
        ..circle(color: theme.colorScheme.onSurface.withOpacity(0.38)),
    );
  });

  testWidgets('Material3 - RadioListTile respects overlayColor in active/pressed/hovered states', (
    WidgetTester tester,
  ) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;

    const Color fillColor = Color(0xFF000000);
    const Color activePressedOverlayColor = Color(0xFF000001);
    const Color inactivePressedOverlayColor = Color(0xFF000002);
    const Color hoverOverlayColor = Color(0xFF000003);
    const Color hoverColor = Color(0xFF000005);

    Color? getOverlayColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.pressed)) {
        if (states.contains(MaterialState.selected)) {
          return activePressedOverlayColor;
        }
        return inactivePressedOverlayColor;
      }
      if (states.contains(MaterialState.hovered)) {
        return hoverOverlayColor;
      }
      return null;
    }

    Widget buildRadio({bool active = false, bool useOverlay = true}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: Material(
          child: RadioListTile<bool>(
            value: active,
            groupValue: true,
            onChanged: (_) {},
            fillColor: const MaterialStatePropertyAll<Color>(fillColor),
            overlayColor: useOverlay ? MaterialStateProperty.resolveWith(getOverlayColor) : null,
            hoverColor: hoverColor,
          ),
        ),
      );
    }

    await tester.pumpWidget(buildRadio(useOverlay: false));
    await tester.press(find.byType(Radio<bool>));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints
        ..rect(color: const Color(0x00000000))
        ..rect(color: const Color(0x66bcbcbc))
        ..circle(color: fillColor.withAlpha(kRadialReactionAlpha), radius: 20.0),
      reason: 'Default inactive pressed Radio should have overlay color from fillColor',
    );

    await tester.pumpWidget(buildRadio(active: true, useOverlay: false));
    await tester.press(find.byType(Radio<bool>));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints
        ..rect(color: const Color(0x00000000))
        ..rect(color: const Color(0x66bcbcbc))
        ..circle(color: fillColor.withAlpha(kRadialReactionAlpha), radius: 20.0),
      reason: 'Default active pressed Radio should have overlay color from fillColor',
    );

    await tester.pumpWidget(buildRadio());
    await tester.press(find.byType(Radio<bool>));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints
        ..rect(color: const Color(0x00000000))
        ..rect(color: const Color(0x66bcbcbc))
        ..circle(color: inactivePressedOverlayColor, radius: 20.0),
      reason: 'Inactive pressed Radio should have overlay color: $inactivePressedOverlayColor',
    );

    await tester.pumpWidget(buildRadio(active: true));
    await tester.press(find.byType(Radio<bool>));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints
        ..rect(color: const Color(0x00000000))
        ..rect(color: const Color(0x66bcbcbc))
        ..circle(color: activePressedOverlayColor, radius: 20.0),
      reason: 'Active pressed Radio should have overlay color: $activePressedOverlayColor',
    );

    // Start hovering.
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byType(Radio<bool>)));
    await tester.pumpAndSettle();

    await tester.pumpWidget(Container());
    await tester.pumpWidget(buildRadio());
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Radio<bool>))),
      paints
        ..rect(color: const Color(0x00000000))
        ..rect(color: const Color(0x0a000000))
        ..circle(color: hoverOverlayColor, radius: 20.0),
      reason: 'Hovered Radio should use overlay color $hoverOverlayColor over $hoverColor',
    );
  });

  testWidgets('RadioListTile respects splashRadius', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    const double splashRadius = 30;
    Widget buildApp() {
      return wrap(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return RadioListTile<int>(
              value: 0,
              onChanged: (_) {},
              hoverColor: Colors.orange[500],
              groupValue: 0,
              splashRadius: splashRadius,
            );
          },
        ),
      );
    }

    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byType(Radio<int>)));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byWidgetPredicate((Widget widget) => widget is Radio<int>))),
      paints..circle(color: Colors.orange[500], radius: splashRadius),
    );
  });

  testWidgets('Radio respects materialTapTargetSize', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        child: RadioListTile<bool>(groupValue: true, value: true, onChanged: (bool? newValue) {}),
      ),
    );

    // default test
    expect(tester.getSize(find.byType(Radio<bool>)), const Size(40.0, 40.0));

    await tester.pumpWidget(
      wrap(
        child: RadioListTile<bool>(
          materialTapTargetSize: MaterialTapTargetSize.padded,
          groupValue: true,
          value: true,
          onChanged: (bool? newValue) {},
        ),
      ),
    );

    expect(tester.getSize(find.byType(Radio<bool>)), const Size(48.0, 48.0));
  });

  testWidgets('RadioListTile.control widget should not request focus on traversal', (
    WidgetTester tester,
  ) async {
    final GlobalKey firstChildKey = GlobalKey();
    final GlobalKey secondChildKey = GlobalKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Column(
            children: <Widget>[
              RadioListTile<bool>(
                value: true,
                groupValue: true,
                onChanged: (bool? value) {},
                title: Text('Hey', key: firstChildKey),
              ),
              RadioListTile<bool>(
                value: true,
                groupValue: true,
                onChanged: (bool? value) {},
                title: Text('There', key: secondChildKey),
              ),
            ],
          ),
        ),
      ),
    );

    await tester.pump();
    Focus.of(firstChildKey.currentContext!).requestFocus();
    await tester.pump();
    expect(Focus.of(firstChildKey.currentContext!).hasPrimaryFocus, isTrue);
    Focus.of(firstChildKey.currentContext!).nextFocus();
    await tester.pump();
    expect(Focus.of(firstChildKey.currentContext!).hasPrimaryFocus, isFalse);
    expect(Focus.of(secondChildKey.currentContext!).hasPrimaryFocus, isTrue);
  });

  testWidgets('RadioListTile.adaptive shows the correct radio platform widget', (
    WidgetTester tester,
  ) async {
    Widget buildApp(TargetPlatform platform) {
      return MaterialApp(
        theme: ThemeData(platform: platform),
        home: Material(
          child: Center(
            child: RadioListTile<int>.adaptive(value: 1, groupValue: 2, onChanged: (_) {}),
          ),
        ),
      );
    }

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    ]) {
      await tester.pumpWidget(buildApp(platform));
      await tester.pumpAndSettle();

      expect(find.byType(CupertinoRadio<int>), findsOneWidget);
    }

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
      TargetPlatform.windows,
    ]) {
      await tester.pumpWidget(buildApp(platform));
      await tester.pumpAndSettle();

      expect(find.byType(CupertinoRadio<int>), findsNothing);
    }
  });

  group('feedback', () {
    late FeedbackTester feedback;

    setUp(() {
      feedback = FeedbackTester();
    });

    tearDown(() {
      feedback.dispose();
    });

    testWidgets('RadioListTile respects enableFeedback', (WidgetTester tester) async {
      const Key key = Key('test');
      Future<void> buildTest(bool enableFeedback) async {
        return tester.pumpWidget(
          wrap(
            child: Center(
              child: RadioListTile<bool>(
                key: key,
                value: false,
                groupValue: true,
                selected: true,
                onChanged: (bool? value) {},
                enableFeedback: enableFeedback,
              ),
            ),
          ),
        );
      }

      await buildTest(false);
      await tester.tap(find.byKey(key));
      await tester.pump(const Duration(seconds: 1));
      expect(feedback.clickSoundCount, 0);
      expect(feedback.hapticCount, 0);

      await buildTest(true);
      await tester.tap(find.byKey(key));
      await tester.pump(const Duration(seconds: 1));
      expect(feedback.clickSoundCount, 1);
      expect(feedback.hapticCount, 0);
    });
  });

  group('Material 2', () {
    // These tests are only relevant for Material 2. Once Material 2
    // support is deprecated and the APIs are removed, these tests
    // can be deleted.

    testWidgets(
      'Material2 - RadioListTile respects overlayColor in active/pressed/hovered states',
      (WidgetTester tester) async {
        tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;

        const Color fillColor = Color(0xFF000000);
        const Color activePressedOverlayColor = Color(0xFF000001);
        const Color inactivePressedOverlayColor = Color(0xFF000002);
        const Color hoverOverlayColor = Color(0xFF000003);
        const Color hoverColor = Color(0xFF000005);

        Color? getOverlayColor(Set<MaterialState> states) {
          if (states.contains(MaterialState.pressed)) {
            if (states.contains(MaterialState.selected)) {
              return activePressedOverlayColor;
            }
            return inactivePressedOverlayColor;
          }
          if (states.contains(MaterialState.hovered)) {
            return hoverOverlayColor;
          }
          return null;
        }

        Widget buildRadio({bool active = false, bool useOverlay = true}) {
          return MaterialApp(
            theme: ThemeData(useMaterial3: false),
            home: Material(
              child: RadioListTile<bool>(
                value: active,
                groupValue: true,
                onChanged: (_) {},
                fillColor: const MaterialStatePropertyAll<Color>(fillColor),
                overlayColor:
                    useOverlay ? MaterialStateProperty.resolveWith(getOverlayColor) : null,
                hoverColor: hoverColor,
              ),
            ),
          );
        }

        await tester.pumpWidget(buildRadio(useOverlay: false));
        await tester.press(find.byType(Radio<bool>));
        await tester.pumpAndSettle();

        expect(
          Material.of(tester.element(find.byType(Radio<bool>))),
          paints
            ..circle()
            ..circle(color: fillColor.withAlpha(kRadialReactionAlpha), radius: 20),
          reason: 'Default inactive pressed Radio should have overlay color from fillColor',
        );

        await tester.pumpWidget(buildRadio(active: true, useOverlay: false));
        await tester.press(find.byType(Radio<bool>));
        await tester.pumpAndSettle();

        expect(
          Material.of(tester.element(find.byType(Radio<bool>))),
          paints
            ..circle()
            ..circle(color: fillColor.withAlpha(kRadialReactionAlpha), radius: 20),
          reason: 'Default active pressed Radio should have overlay color from fillColor',
        );

        await tester.pumpWidget(buildRadio());
        await tester.press(find.byType(Radio<bool>));
        await tester.pumpAndSettle();

        expect(
          Material.of(tester.element(find.byType(Radio<bool>))),
          paints
            ..circle()
            ..circle(color: inactivePressedOverlayColor, radius: 20),
          reason: 'Inactive pressed Radio should have overlay color: $inactivePressedOverlayColor',
        );

        // Start hovering.
        final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
        await gesture.addPointer();
        await gesture.moveTo(tester.getCenter(find.byType(Radio<bool>)));
        await tester.pumpAndSettle();

        await tester.pumpWidget(Container());
        await tester.pumpWidget(buildRadio());
        await tester.pumpAndSettle();

        expect(
          Material.of(tester.element(find.byType(Radio<bool>))),
          paints..circle(color: hoverOverlayColor, radius: 20),
          reason: 'Hovered Radio should use overlay color $hoverOverlayColor over $hoverColor',
        );
      },
    );

    testWidgets('Material2 - RadioListTile respects hoverColor', (WidgetTester tester) async {
      tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
      int? groupValue = 0;
      final Color? hoverColor = Colors.orange[500];
      Widget buildApp({bool enabled = true}) {
        return wrap(
          child: MaterialApp(
            theme: ThemeData(useMaterial3: false),
            home: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return RadioListTile<int>(
                  value: 0,
                  onChanged:
                      enabled
                          ? (int? newValue) {
                            setState(() {
                              groupValue = newValue;
                            });
                          }
                          : null,
                  hoverColor: hoverColor,
                  groupValue: groupValue,
                );
              },
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp());

      await tester.pump();
      await tester.pumpAndSettle();
      expect(
        Material.of(tester.element(find.byType(Radio<int>))),
        paints
          ..rect()
          ..circle(color: const Color(0xff2196f3))
          ..circle(color: const Color(0xff2196f3)),
      );

      // Start hovering
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.moveTo(tester.getCenter(find.byType(Radio<int>)));

      // Check when the radio isn't selected.
      groupValue = 1;
      await tester.pumpWidget(buildApp());
      await tester.pump();
      await tester.pumpAndSettle();
      expect(
        Material.of(tester.element(find.byType(Radio<int>))),
        paints
          ..rect()
          ..circle(color: hoverColor),
      );

      // Check when the radio is selected, but disabled.
      groupValue = 0;
      await tester.pumpWidget(buildApp(enabled: false));
      await tester.pump();
      await tester.pumpAndSettle();
      expect(
        Material.of(tester.element(find.byType(Radio<int>))),
        paints
          ..rect()
          ..circle(color: const Color(0x61000000))
          ..circle(color: const Color(0x61000000)),
      );
    });
  });

  testWidgets('RadioListTile uses ListTileTheme controlAffinity', (WidgetTester tester) async {
    Widget buildListTile(ListTileControlAffinity controlAffinity) {
      return MaterialApp(
        home: Material(
          child: ListTileTheme(
            data: ListTileThemeData(controlAffinity: controlAffinity),
            child: RadioListTile<double>(
              value: 0.5,
              groupValue: 1.0,
              title: const Text('RadioListTile'),
              onChanged: (double? value) {},
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildListTile(ListTileControlAffinity.leading));
    final Finder leading = find.text('RadioListTile');
    final Offset offsetLeading = tester.getTopLeft(leading);
    expect(offsetLeading, const Offset(72.0, 16.0));

    await tester.pumpWidget(buildListTile(ListTileControlAffinity.trailing));
    final Finder trailing = find.text('RadioListTile');
    final Offset offsetTrailing = tester.getTopLeft(trailing);
    expect(offsetTrailing, const Offset(16.0, 16.0));

    await tester.pumpWidget(buildListTile(ListTileControlAffinity.platform));
    final Finder platform = find.text('RadioListTile');
    final Offset offsetPlatform = tester.getTopLeft(platform);
    expect(offsetPlatform, const Offset(72.0, 16.0));
  });

  testWidgets('RadioListTile renders with default scale', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: RadioListTile<bool>(value: false, groupValue: false, onChanged: null),
        ),
      ),
    );

    final Finder transformFinder = find.ancestor(
      of: find.byType(Radio<bool>),
      matching: find.byType(Transform),
    );

    expect(transformFinder, findsNothing);
  });

  testWidgets('RadioListTile respects radioScaleFactor', (WidgetTester tester) async {
    const double scale = 1.4;
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: RadioListTile<bool>(
            value: false,
            groupValue: false,
            onChanged: null,
            radioScaleFactor: scale,
          ),
        ),
      ),
    );

    final Transform widget = tester.widget(
      find.ancestor(of: find.byType(Radio<bool>), matching: find.byType(Transform)),
    );

    expect(widget.transform.getMaxScaleOnAxis(), scale);
  });
}
