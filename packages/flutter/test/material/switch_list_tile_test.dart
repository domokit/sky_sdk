// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/feedback_tester.dart';
import '../widgets/semantics_tester.dart';

Widget wrap({required Widget child}) {
  return MediaQuery(
    data: const MediaQueryData(),
    child: Directionality(textDirection: TextDirection.ltr, child: Material(child: child)),
  );
}

void main() {
  testWidgets('SwitchListTile control test', (WidgetTester tester) async {
    final List<dynamic> log = <dynamic>[];
    await tester.pumpWidget(
      wrap(
        child: SwitchListTile(
          value: true,
          onChanged: (bool value) {
            log.add(value);
          },
          title: const Text('Hello'),
        ),
      ),
    );
    await tester.tap(find.text('Hello'));
    log.add('-');
    await tester.tap(find.byType(Switch));
    expect(log, equals(<dynamic>[false, '-', false]));
  });

  testWidgets('SwitchListTile semantics test', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      wrap(
        child: Column(
          children: <Widget>[
            SwitchListTile(
              value: true,
              onChanged: (bool value) {},
              title: const Text('AAA'),
              secondary: const Text('aaa'),
              internalAddSemanticForOnTap: true,
            ),
            CheckboxListTile(
              value: true,
              onChanged: (bool? value) {},
              title: const Text('BBB'),
              secondary: const Text('bbb'),
              internalAddSemanticForOnTap: true,
            ),
            RadioListTile<bool>(
              value: true,
              groupValue: false,
              onChanged: (bool? value) {},
              title: const Text('CCC'),
              secondary: const Text('ccc'),
              internalAddSemanticForOnTap: true,
            ),
          ],
        ),
      ),
    );

    // This test verifies that the label and the control get merged.
    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              id: 1,
              rect: const Rect.fromLTWH(0.0, 0.0, 800.0, 56.0),
              flags: <SemanticsFlag>[
                SemanticsFlag.isButton,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.hasToggledState,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isFocusable,
                SemanticsFlag.isToggled,
                SemanticsFlag.hasSelectedState,
              ],
              actions: SemanticsAction.tap.index | SemanticsAction.focus.index,
              label: 'aaa\nAAA',
            ),
            TestSemantics.rootChild(
              id: 3,
              rect: const Rect.fromLTWH(0.0, 0.0, 800.0, 56.0),
              transform: Matrix4.translationValues(0.0, 56.0, 0.0),
              flags: <SemanticsFlag>[
                SemanticsFlag.isButton,
                SemanticsFlag.hasCheckedState,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isChecked,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isFocusable,
                SemanticsFlag.hasSelectedState,
              ],
              actions: SemanticsAction.tap.index | SemanticsAction.focus.index,
              label: 'bbb\nBBB',
            ),
            TestSemantics.rootChild(
              id: 5,
              rect: const Rect.fromLTWH(0.0, 0.0, 800.0, 56.0),
              transform: Matrix4.translationValues(0.0, 112.0, 0.0),
              flags: <SemanticsFlag>[
                SemanticsFlag.isButton,
                SemanticsFlag.hasCheckedState,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isFocusable,
                SemanticsFlag.isInMutuallyExclusiveGroup,
                SemanticsFlag.hasSelectedState,
              ],
              actions: SemanticsAction.tap.index | SemanticsAction.focus.index,
              label: 'CCC\nccc',
            ),
          ],
        ),
      ),
    );

    semantics.dispose();
  });

  testWidgets('Material2 - SwitchListTile has the right colors', (WidgetTester tester) async {
    bool value = false;
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(padding: EdgeInsets.all(8.0)),
        child: Theme(
          data: ThemeData(useMaterial3: false),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Material(
                  child: SwitchListTile(
                    value: value,
                    onChanged: (bool newValue) {
                      setState(() {
                        value = newValue;
                      });
                    },
                    activeColor: Colors.red[500],
                    activeTrackColor: Colors.green[500],
                    inactiveThumbColor: Colors.yellow[500],
                    inactiveTrackColor: Colors.blue[500],
                  ),
                );
              },
            ),
          ),
        ),
      ),
    );

    expect(
      find.byType(Switch),
      paints
        ..rrect(color: Colors.blue[500])
        ..rrect()
        ..rrect(color: const Color(0x33000000))
        ..rrect(color: const Color(0x24000000))
        ..rrect(color: const Color(0x1f000000))
        ..rrect(color: Colors.yellow[500]),
    );

    await tester.tap(find.byType(Switch));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(color: Colors.green[500])
        ..rrect()
        ..rrect(color: const Color(0x33000000))
        ..rrect(color: const Color(0x24000000))
        ..rrect(color: const Color(0x1f000000))
        ..rrect(color: Colors.red[500]),
    );
  });

  testWidgets('Material3 - SwitchListTile has the right colors', (WidgetTester tester) async {
    bool value = false;
    await tester.pumpWidget(
      MediaQuery(
        data: const MediaQueryData(padding: EdgeInsets.all(8.0)),
        child: Theme(
          data: ThemeData(useMaterial3: true),
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Material(
                  child: SwitchListTile(
                    value: value,
                    onChanged: (bool newValue) {
                      setState(() {
                        value = newValue;
                      });
                    },
                    activeColor: Colors.red[500],
                    activeTrackColor: Colors.green[500],
                    inactiveThumbColor: Colors.yellow[500],
                    inactiveTrackColor: Colors.blue[500],
                  ),
                );
              },
            ),
          ),
        ),
      ),
    );

    expect(
      find.byType(Switch),
      paints
        ..rrect(color: Colors.blue[500])
        ..rrect()
        ..rrect(color: Colors.yellow[500]),
    );

    await tester.tap(find.byType(Switch));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(color: Colors.green[500])
        ..rrect()
        ..rrect(color: Colors.red[500]),
    );
  });

  testWidgets('SwitchListTile.adaptive only uses material switch', (WidgetTester tester) async {
    bool value = false;

    Widget buildFrame(TargetPlatform platform) {
      return MaterialApp(
        theme: ThemeData(platform: platform),
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Material(
              child: Center(
                child: SwitchListTile.adaptive(
                  value: value,
                  onChanged: (bool newValue) {
                    setState(() {
                      value = newValue;
                    });
                  },
                ),
              ),
            );
          },
        ),
      );
    }

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.iOS,
      TargetPlatform.macOS,
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
      TargetPlatform.windows,
    ]) {
      value = false;
      await tester.pumpWidget(buildFrame(platform));
      expect(find.byType(CupertinoSwitch), findsNothing);
      expect(find.byType(Switch), findsOneWidget);
      expect(value, isFalse, reason: 'on ${platform.name}');

      await tester.tap(find.byType(SwitchListTile));
      expect(value, isTrue, reason: 'on ${platform.name}');
    }
  });

  testWidgets('SwitchListTile contentPadding', (WidgetTester tester) async {
    Widget buildFrame(TextDirection textDirection) {
      return MediaQuery(
        data: const MediaQueryData(),
        child: Directionality(
          textDirection: textDirection,
          child: Material(
            child: Container(
              alignment: Alignment.topLeft,
              child: SwitchListTile(
                contentPadding: const EdgeInsetsDirectional.only(
                  start: 10.0,
                  end: 20.0,
                  top: 30.0,
                  bottom: 40.0,
                ),
                secondary: const Text('L'),
                title: const Text('title'),
                value: true,
                onChanged: (bool selected) {},
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(TextDirection.ltr));

    expect(tester.getTopLeft(find.text('L')).dx, 10.0); // contentPadding.start = 10
    expect(tester.getTopRight(find.byType(Switch)).dx, 780.0); // 800 - contentPadding.end

    await tester.pumpWidget(buildFrame(TextDirection.rtl));

    expect(tester.getTopLeft(find.byType(Switch)).dx, 20.0); // contentPadding.end = 20
    expect(tester.getTopRight(find.text('L')).dx, 790.0); // 800 - contentPadding.start
  });

  testWidgets('SwitchListTile can autofocus unless disabled.', (WidgetTester tester) async {
    final GlobalKey childKey = GlobalKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: ListView(
            children: <Widget>[
              SwitchListTile(
                value: true,
                onChanged: (_) {},
                title: Text('A', key: childKey),
                autofocus: true,
              ),
            ],
          ),
        ),
      ),
    );

    await tester.pump();
    expect(Focus.of(childKey.currentContext!).hasPrimaryFocus, isTrue);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: ListView(
            children: <Widget>[
              SwitchListTile(
                value: true,
                onChanged: null,
                title: Text('A', key: childKey),
                autofocus: true,
              ),
            ],
          ),
        ),
      ),
    );

    await tester.pump();
    expect(Focus.of(childKey.currentContext!).hasPrimaryFocus, isFalse);
  });

  testWidgets('SwitchListTile controlAffinity test', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: SwitchListTile(
            value: true,
            onChanged: null,
            secondary: Icon(Icons.info),
            title: Text('Title'),
            controlAffinity: ListTileControlAffinity.leading,
          ),
        ),
      ),
    );

    final ListTile listTile = tester.widget(find.byType(ListTile));
    // When controlAffinity is ListTileControlAffinity.leading, the position of
    // Switch is at leading edge and SwitchListTile.secondary at trailing edge.

    // Find the ExcludeFocus widget within the ListTile's leading
    final ExcludeFocus excludeFocusWidget = tester.widget(
      find.byWidgetPredicate(
        (Widget widget) => listTile.leading == widget && widget is ExcludeFocus,
      ),
    );

    // Assert that the ExcludeFocus widget is not null
    expect(excludeFocusWidget, isNotNull);

    // Assert that the child of ExcludeFocus is Switch
    expect(excludeFocusWidget.child.runtimeType, Switch);

    // Assert that the trailing is Icon
    expect(listTile.trailing.runtimeType, Icon);
  });

  testWidgets('SwitchListTile controlAffinity default value test', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: SwitchListTile(
            value: true,
            onChanged: null,
            secondary: Icon(Icons.info),
            title: Text('Title'),
          ),
        ),
      ),
    );

    final ListTile listTile = tester.widget(find.byType(ListTile));
    // By default, value of controlAffinity is ListTileControlAffinity.platform,
    // where the position of SwitchListTile.secondary is at leading edge and Switch
    // at trailing edge. This also covers test for ListTileControlAffinity.trailing.

    // Find the ExcludeFocus widget within the ListTile's trailing
    final ExcludeFocus excludeFocusWidget = tester.widget(
      find.byWidgetPredicate(
        (Widget widget) => listTile.trailing == widget && widget is ExcludeFocus,
      ),
    );

    // Assert that the ExcludeFocus widget is not null
    expect(excludeFocusWidget, isNotNull);

    // Assert that the child of ExcludeFocus is Switch
    expect(excludeFocusWidget.child.runtimeType, Switch);

    // Assert that the leading is Icon
    expect(listTile.leading.runtimeType, Icon);
  });

  testWidgets('SwitchListTile respects shape', (WidgetTester tester) async {
    const ShapeBorder shapeBorder = RoundedRectangleBorder(
      borderRadius: BorderRadius.horizontal(right: Radius.circular(100)),
    );

    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: SwitchListTile(
            value: true,
            onChanged: null,
            title: Text('Title'),
            shape: shapeBorder,
          ),
        ),
      ),
    );

    expect(tester.widget<InkWell>(find.byType(InkWell)).customBorder, shapeBorder);
  });

  testWidgets('SwitchListTile respects tileColor', (WidgetTester tester) async {
    final Color tileColor = Colors.red.shade500;

    await tester.pumpWidget(
      wrap(
        child: Center(
          child: SwitchListTile(
            value: false,
            onChanged: null,
            title: const Text('Title'),
            tileColor: tileColor,
          ),
        ),
      ),
    );

    expect(find.byType(Material), paints..rect(color: tileColor));
  });

  testWidgets('SwitchListTile respects selectedTileColor', (WidgetTester tester) async {
    final Color selectedTileColor = Colors.green.shade500;

    await tester.pumpWidget(
      wrap(
        child: Center(
          child: SwitchListTile(
            value: false,
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

  testWidgets('SwitchListTile selected item text Color', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/pull/76909

    const Color activeColor = Color(0xff00ff00);

    Widget buildFrame({Color? activeColor, Color? thumbColor}) {
      return MaterialApp(
        theme: ThemeData.light().copyWith(
          switchTheme: SwitchThemeData(
            thumbColor: MaterialStateProperty.resolveWith<Color?>((Set<MaterialState> states) {
              return states.contains(MaterialState.selected) ? thumbColor : null;
            }),
          ),
        ),
        home: Scaffold(
          body: Center(
            child: SwitchListTile(
              activeColor: activeColor,
              selected: true,
              title: const Text('title'),
              value: true,
              onChanged: (bool? value) {},
            ),
          ),
        ),
      );
    }

    Color? textColor(String text) {
      return tester.renderObject<RenderParagraph>(find.text(text)).text.style?.color;
    }

    await tester.pumpWidget(buildFrame(activeColor: activeColor));
    expect(textColor('title'), activeColor);

    await tester.pumpWidget(buildFrame(activeColor: activeColor));
    expect(textColor('title'), activeColor);
  });

  testWidgets('SwitchListTile respects visualDensity', (WidgetTester tester) async {
    const Key key = Key('test');
    Future<void> buildTest(VisualDensity visualDensity) async {
      return tester.pumpWidget(
        wrap(
          child: Center(
            child: SwitchListTile(
              key: key,
              value: false,
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

  testWidgets('SwitchListTile respects focusNode', (WidgetTester tester) async {
    final GlobalKey childKey = GlobalKey();
    await tester.pumpWidget(
      wrap(
        child: Center(
          child: SwitchListTile(
            value: false,
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

  testWidgets('SwitchListTile onFocusChange callback', (WidgetTester tester) async {
    final FocusNode node = FocusNode(debugLabel: 'SwitchListTile onFocusChange');
    bool gotFocus = false;
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SwitchListTile(
            value: true,
            focusNode: node,
            onFocusChange: (bool focused) {
              gotFocus = focused;
            },
            onChanged: (bool value) {},
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
    node.dispose();
  });

  testWidgets('SwitchListTile.adaptive onFocusChange Callback', (WidgetTester tester) async {
    final FocusNode node = FocusNode(debugLabel: 'SwitchListTile.adaptive onFocusChange');
    bool gotFocus = false;
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: SwitchListTile.adaptive(
            value: true,
            focusNode: node,
            onFocusChange: (bool focused) {
              gotFocus = focused;
            },
            onChanged: (bool value) {},
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
    node.dispose();
  });

  group('feedback', () {
    late FeedbackTester feedback;

    setUp(() {
      feedback = FeedbackTester();
    });

    tearDown(() {
      feedback.dispose();
    });

    testWidgets('SwitchListTile respects enableFeedback', (WidgetTester tester) async {
      Future<void> buildTest(bool enableFeedback) async {
        return tester.pumpWidget(
          wrap(
            child: Center(
              child: SwitchListTile(
                value: false,
                onChanged: (bool? value) {},
                enableFeedback: enableFeedback,
              ),
            ),
          ),
        );
      }

      await buildTest(false);
      await tester.tap(find.byType(SwitchListTile));
      await tester.pump(const Duration(seconds: 1));
      expect(feedback.clickSoundCount, 0);
      expect(feedback.hapticCount, 0);

      await buildTest(true);
      await tester.tap(find.byType(SwitchListTile));
      await tester.pump(const Duration(seconds: 1));
      expect(feedback.clickSoundCount, 1);
      expect(feedback.hapticCount, 0);
    });
  });

  testWidgets('SwitchListTile respects hoverColor', (WidgetTester tester) async {
    const Key key = Key('test');
    await tester.pumpWidget(
      wrap(
        child: Center(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Container(
                width: 500,
                height: 100,
                color: Colors.white,
                child: SwitchListTile(
                  value: false,
                  key: key,
                  hoverColor: Colors.orange[500],
                  title: const Text('A'),
                  onChanged: (bool? value) {},
                ),
              );
            },
          ),
        ),
      ),
    );

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.moveTo(tester.getCenter(find.byKey(key)));

    await tester.pump();
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byKey(key))),
      paints
        ..rect()
        ..rect(color: Colors.orange[500], rect: const Rect.fromLTRB(150.0, 250.0, 650.0, 350.0)),
    );
  });

  testWidgets('Material2 - SwitchListTile respects thumbColor in active/enabled states', (
    WidgetTester tester,
  ) async {
    const Color activeEnabledThumbColor = Color(0xFF000001);
    const Color activeDisabledThumbColor = Color(0xFF000002);
    const Color inactiveEnabledThumbColor = Color(0xFF000003);
    const Color inactiveDisabledThumbColor = Color(0xFF000004);

    Color getThumbColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        if (states.contains(MaterialState.selected)) {
          return activeDisabledThumbColor;
        }
        return inactiveDisabledThumbColor;
      }
      if (states.contains(MaterialState.selected)) {
        return activeEnabledThumbColor;
      }
      return inactiveEnabledThumbColor;
    }

    final MaterialStateProperty<Color> thumbColor = MaterialStateColor.resolveWith(getThumbColor);

    Widget buildSwitchListTile({required bool enabled, required bool selected}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return SwitchListTile(
                value: selected,
                thumbColor: thumbColor,
                onChanged: enabled ? (_) {} : null,
              );
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSwitchListTile(enabled: false, selected: false));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect(color: inactiveDisabledThumbColor),
    );

    await tester.pumpWidget(buildSwitchListTile(enabled: false, selected: true));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect(color: activeDisabledThumbColor),
    );

    await tester.pumpWidget(buildSwitchListTile(enabled: true, selected: false));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect(color: inactiveEnabledThumbColor),
    );

    await tester.pumpWidget(buildSwitchListTile(enabled: true, selected: true));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect(color: activeEnabledThumbColor),
    );
  });

  testWidgets('Material3 - SwitchListTile respects thumbColor in active/enabled states', (
    WidgetTester tester,
  ) async {
    const Color activeEnabledThumbColor = Color(0xFF000001);
    const Color activeDisabledThumbColor = Color(0xFF000002);
    const Color inactiveEnabledThumbColor = Color(0xFF000003);
    const Color inactiveDisabledThumbColor = Color(0xFF000004);

    Color getThumbColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        if (states.contains(MaterialState.selected)) {
          return activeDisabledThumbColor;
        }
        return inactiveDisabledThumbColor;
      }
      if (states.contains(MaterialState.selected)) {
        return activeEnabledThumbColor;
      }
      return inactiveEnabledThumbColor;
    }

    final MaterialStateProperty<Color> thumbColor = MaterialStateColor.resolveWith(getThumbColor);

    Widget buildSwitchListTile({required bool enabled, required bool selected}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: Material(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return SwitchListTile(
                value: selected,
                thumbColor: thumbColor,
                onChanged: enabled ? (_) {} : null,
              );
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSwitchListTile(enabled: false, selected: false));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..rrect()
        ..rrect(color: inactiveDisabledThumbColor),
    );

    await tester.pumpWidget(buildSwitchListTile(enabled: false, selected: true));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..rrect()
        ..rrect(color: activeDisabledThumbColor),
    );

    await tester.pumpWidget(buildSwitchListTile(enabled: true, selected: false));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..rrect()
        ..rrect(color: inactiveEnabledThumbColor),
    );

    await tester.pumpWidget(buildSwitchListTile(enabled: true, selected: true));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..rrect()
        ..rrect(color: activeEnabledThumbColor),
    );
  });

  testWidgets('Material2 - SwitchListTile respects thumbColor in hovered/pressed states', (
    WidgetTester tester,
  ) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    const Color hoveredThumbColor = Color(0xFF4caf50);
    const Color pressedThumbColor = Color(0xFFF44336);

    Color getThumbColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.pressed)) {
        return pressedThumbColor;
      }
      if (states.contains(MaterialState.hovered)) {
        return hoveredThumbColor;
      }
      return Colors.transparent;
    }

    final MaterialStateProperty<Color> thumbColor = MaterialStateColor.resolveWith(getThumbColor);

    Widget buildSwitchListTile() {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return SwitchListTile(value: false, thumbColor: thumbColor, onChanged: (_) {});
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSwitchListTile());
    await tester.pumpAndSettle();

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.moveTo(tester.getCenter(find.byType(Switch)));

    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect(color: hoveredThumbColor),
    );

    // On pressed state
    await tester.press(find.byType(Switch));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect()
        ..rrect(color: pressedThumbColor),
    );
  });

  testWidgets('Material3 - SwitchListTile respects thumbColor in hovered/pressed states', (
    WidgetTester tester,
  ) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    const Color hoveredThumbColor = Color(0xFF4caf50);
    const Color pressedThumbColor = Color(0xFFF44336);

    Color getThumbColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.pressed)) {
        return pressedThumbColor;
      }
      if (states.contains(MaterialState.hovered)) {
        return hoveredThumbColor;
      }
      return Colors.transparent;
    }

    final MaterialStateProperty<Color> thumbColor = MaterialStateColor.resolveWith(getThumbColor);

    Widget buildSwitchListTile() {
      return MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: Material(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return SwitchListTile(value: false, thumbColor: thumbColor, onChanged: (_) {});
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSwitchListTile());
    await tester.pumpAndSettle();

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.moveTo(tester.getCenter(find.byType(Switch)));

    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..rrect()
        ..rrect(color: hoveredThumbColor),
    );

    // On pressed state
    await tester.press(find.byType(Switch));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..rrect()
        ..rrect(color: pressedThumbColor),
    );
  });

  testWidgets('SwitchListTile respects trackColor in active/enabled states', (
    WidgetTester tester,
  ) async {
    const Color activeEnabledTrackColor = Color(0xFF000001);
    const Color activeDisabledTrackColor = Color(0xFF000002);
    const Color inactiveEnabledTrackColor = Color(0xFF000003);
    const Color inactiveDisabledTrackColor = Color(0xFF000004);

    Color getTrackColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        if (states.contains(MaterialState.selected)) {
          return activeDisabledTrackColor;
        }
        return inactiveDisabledTrackColor;
      }
      if (states.contains(MaterialState.selected)) {
        return activeEnabledTrackColor;
      }
      return inactiveEnabledTrackColor;
    }

    final MaterialStateProperty<Color> trackColor = MaterialStateColor.resolveWith(getTrackColor);

    Widget buildSwitchListTile({required bool enabled, required bool selected}) {
      return wrap(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return SwitchListTile(
              value: selected,
              trackColor: trackColor,
              onChanged: enabled ? (_) {} : null,
            );
          },
        ),
      );
    }

    await tester.pumpWidget(buildSwitchListTile(enabled: false, selected: false));

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints..rrect(color: inactiveDisabledTrackColor),
    );

    await tester.pumpWidget(buildSwitchListTile(enabled: false, selected: true));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints..rrect(color: activeDisabledTrackColor),
    );

    await tester.pumpWidget(buildSwitchListTile(enabled: true, selected: false));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints..rrect(color: inactiveEnabledTrackColor),
    );

    await tester.pumpWidget(buildSwitchListTile(enabled: true, selected: true));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints..rrect(color: activeEnabledTrackColor),
    );
  });

  testWidgets('SwitchListTile respects trackColor in hovered states', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    const Color hoveredTrackColor = Color(0xFF4caf50);

    Color getTrackColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.hovered)) {
        return hoveredTrackColor;
      }
      return Colors.transparent;
    }

    final MaterialStateProperty<Color> trackColor = MaterialStateColor.resolveWith(getTrackColor);

    Widget buildSwitchListTile() {
      return wrap(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return SwitchListTile(value: false, trackColor: trackColor, onChanged: (_) {});
          },
        ),
      );
    }

    await tester.pumpWidget(buildSwitchListTile());
    await tester.pumpAndSettle();

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.moveTo(tester.getCenter(find.byType(Switch)));

    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints..rrect(color: hoveredTrackColor),
    );
  });

  testWidgets('SwitchListTile respects thumbIcon - M3', (WidgetTester tester) async {
    const Icon activeIcon = Icon(Icons.check);
    const Icon inactiveIcon = Icon(Icons.close);

    MaterialStateProperty<Icon?> thumbIcon(Icon? activeIcon, Icon? inactiveIcon) {
      return MaterialStateProperty.resolveWith<Icon?>((Set<MaterialState> states) {
        if (states.contains(MaterialState.selected)) {
          return activeIcon;
        }
        return inactiveIcon;
      });
    }

    Widget buildSwitchListTile({
      required bool enabled,
      required bool active,
      Icon? activeIcon,
      Icon? inactiveIcon,
    }) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: wrap(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return SwitchListTile(
                thumbIcon: thumbIcon(activeIcon, inactiveIcon),
                value: active,
                onChanged: enabled ? (_) {} : null,
              );
            },
          ),
        ),
      );
    }

    // active icon shows when switch is on.
    await tester.pumpWidget(
      buildSwitchListTile(enabled: true, active: true, activeIcon: activeIcon),
    );
    await tester.pumpAndSettle();
    final Switch switchWidget0 = tester.widget<Switch>(find.byType(Switch));
    expect(switchWidget0.thumbIcon?.resolve(<MaterialState>{MaterialState.selected}), activeIcon);
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..rrect()
        ..paragraph(offset: const Offset(32.0, 12.0)),
    );

    // inactive icon shows when switch is off.
    await tester.pumpWidget(
      buildSwitchListTile(enabled: true, active: false, inactiveIcon: inactiveIcon),
    );
    await tester.pumpAndSettle();
    final Switch switchWidget1 = tester.widget<Switch>(find.byType(Switch));
    expect(switchWidget1.thumbIcon?.resolve(<MaterialState>{}), inactiveIcon);
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..rrect()
        ..rrect()
        ..paragraph(offset: const Offset(12.0, 12.0)),
    );

    // active icon doesn't show when switch is off.
    await tester.pumpWidget(
      buildSwitchListTile(enabled: true, active: false, activeIcon: activeIcon),
    );
    await tester.pumpAndSettle();
    final Switch switchWidget2 = tester.widget<Switch>(find.byType(Switch));
    expect(switchWidget2.thumbIcon?.resolve(<MaterialState>{MaterialState.selected}), activeIcon);
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..rrect()
        ..rrect(),
    );

    // inactive icon doesn't show when switch is on.
    await tester.pumpWidget(
      buildSwitchListTile(enabled: true, active: true, inactiveIcon: inactiveIcon),
    );
    await tester.pumpAndSettle();
    final Switch switchWidget3 = tester.widget<Switch>(find.byType(Switch));
    expect(switchWidget3.thumbIcon?.resolve(<MaterialState>{}), inactiveIcon);
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..rrect()
        ..restore(),
    );

    // without icon
    await tester.pumpWidget(buildSwitchListTile(enabled: true, active: false));
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..rrect()
        ..rrect()
        ..restore(),
    );
  });

  testWidgets('Material2 - SwitchListTile respects materialTapTargetSize', (
    WidgetTester tester,
  ) async {
    Widget buildSwitchListTile(MaterialTapTargetSize materialTapTargetSize) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return SwitchListTile(
                materialTapTargetSize: materialTapTargetSize,
                value: false,
                onChanged: (_) {},
              );
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSwitchListTile(MaterialTapTargetSize.padded));
    final Switch switchWidget = tester.widget<Switch>(find.byType(Switch));
    expect(switchWidget.materialTapTargetSize, MaterialTapTargetSize.padded);
    expect(tester.getSize(find.byType(Switch)), const Size(59.0, 48.0));

    await tester.pumpWidget(buildSwitchListTile(MaterialTapTargetSize.shrinkWrap));
    final Switch switchWidget1 = tester.widget<Switch>(find.byType(Switch));
    expect(switchWidget1.materialTapTargetSize, MaterialTapTargetSize.shrinkWrap);
    expect(tester.getSize(find.byType(Switch)), const Size(59.0, 40.0));
  });

  testWidgets('Material3 - SwitchListTile respects materialTapTargetSize', (
    WidgetTester tester,
  ) async {
    Widget buildSwitchListTile(MaterialTapTargetSize materialTapTargetSize) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: Material(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return SwitchListTile(
                materialTapTargetSize: materialTapTargetSize,
                value: false,
                onChanged: (_) {},
              );
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSwitchListTile(MaterialTapTargetSize.padded));
    final Switch switchWidget = tester.widget<Switch>(find.byType(Switch));
    expect(switchWidget.materialTapTargetSize, MaterialTapTargetSize.padded);
    expect(tester.getSize(find.byType(Switch)), const Size(60.0, 48.0));

    await tester.pumpWidget(buildSwitchListTile(MaterialTapTargetSize.shrinkWrap));
    final Switch switchWidget1 = tester.widget<Switch>(find.byType(Switch));
    expect(switchWidget1.materialTapTargetSize, MaterialTapTargetSize.shrinkWrap);
    expect(tester.getSize(find.byType(Switch)), const Size(60.0, 40.0));
  });

  testWidgets('Material2 - SwitchListTile.adaptive respects applyCupertinoTheme', (
    WidgetTester tester,
  ) async {
    Widget buildSwitchListTile(bool applyCupertinoTheme, TargetPlatform platform) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false, platform: platform),
        home: Material(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return SwitchListTile.adaptive(
                applyCupertinoTheme: applyCupertinoTheme,
                value: true,
                onChanged: (_) {},
              );
            },
          ),
        ),
      );
    }

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    ]) {
      await tester.pumpWidget(buildSwitchListTile(true, platform));
      await tester.pumpAndSettle();
      expect(find.byType(Switch), findsOneWidget);
      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints..rrect(color: const Color(0xFF2196F3)),
      );

      await tester.pumpWidget(buildSwitchListTile(false, platform));
      await tester.pumpAndSettle();
      expect(find.byType(Switch), findsOneWidget);
      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints..rrect(color: const Color(0xFF34C759)),
      );
    }
  });

  testWidgets('Material3 - SwitchListTile.adaptive respects applyCupertinoTheme', (
    WidgetTester tester,
  ) async {
    Widget buildSwitchListTile(bool applyCupertinoTheme, TargetPlatform platform) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: true, platform: platform),
        home: Material(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return SwitchListTile.adaptive(
                applyCupertinoTheme: applyCupertinoTheme,
                value: true,
                onChanged: (_) {},
              );
            },
          ),
        ),
      );
    }

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    ]) {
      await tester.pumpWidget(buildSwitchListTile(true, platform));
      await tester.pumpAndSettle();
      expect(find.byType(Switch), findsOneWidget);
      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints..rrect(color: const Color(0xFF6750A4)),
      );

      await tester.pumpWidget(buildSwitchListTile(false, platform));
      await tester.pumpAndSettle();
      expect(find.byType(Switch), findsOneWidget);
      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints..rrect(color: const Color(0xFF34C759)),
      );
    }
  });

  testWidgets('Material2 - SwitchListTile respects materialTapTargetSize', (
    WidgetTester tester,
  ) async {
    Widget buildSwitchListTile(MaterialTapTargetSize materialTapTargetSize) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return SwitchListTile(
                materialTapTargetSize: materialTapTargetSize,
                value: false,
                onChanged: (_) {},
              );
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSwitchListTile(MaterialTapTargetSize.padded));
    final Switch switchWidget = tester.widget<Switch>(find.byType(Switch));
    expect(switchWidget.materialTapTargetSize, MaterialTapTargetSize.padded);
    expect(tester.getSize(find.byType(Switch)), const Size(59.0, 48.0));

    await tester.pumpWidget(buildSwitchListTile(MaterialTapTargetSize.shrinkWrap));
    final Switch switchWidget1 = tester.widget<Switch>(find.byType(Switch));
    expect(switchWidget1.materialTapTargetSize, MaterialTapTargetSize.shrinkWrap);
    expect(tester.getSize(find.byType(Switch)), const Size(59.0, 40.0));
  });

  testWidgets('Material3 - SwitchListTile respects materialTapTargetSize', (
    WidgetTester tester,
  ) async {
    Widget buildSwitchListTile(MaterialTapTargetSize materialTapTargetSize) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: Material(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return SwitchListTile(
                materialTapTargetSize: materialTapTargetSize,
                value: false,
                onChanged: (_) {},
              );
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSwitchListTile(MaterialTapTargetSize.padded));
    final Switch switchWidget = tester.widget<Switch>(find.byType(Switch));
    expect(switchWidget.materialTapTargetSize, MaterialTapTargetSize.padded);
    expect(tester.getSize(find.byType(Switch)), const Size(60.0, 48.0));

    await tester.pumpWidget(buildSwitchListTile(MaterialTapTargetSize.shrinkWrap));
    final Switch switchWidget1 = tester.widget<Switch>(find.byType(Switch));
    expect(switchWidget1.materialTapTargetSize, MaterialTapTargetSize.shrinkWrap);
    expect(tester.getSize(find.byType(Switch)), const Size(60.0, 40.0));
  });

  testWidgets('SwitchListTile passes the value of dragStartBehavior to Switch', (
    WidgetTester tester,
  ) async {
    Widget buildSwitchListTile(DragStartBehavior dragStartBehavior) {
      return wrap(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return SwitchListTile(
              dragStartBehavior: dragStartBehavior,
              value: false,
              onChanged: (_) {},
            );
          },
        ),
      );
    }

    await tester.pumpWidget(buildSwitchListTile(DragStartBehavior.start));
    final Switch switchWidget = tester.widget<Switch>(find.byType(Switch));
    expect(switchWidget.dragStartBehavior, DragStartBehavior.start);

    await tester.pumpWidget(buildSwitchListTile(DragStartBehavior.down));
    final Switch switchWidget1 = tester.widget<Switch>(find.byType(Switch));
    expect(switchWidget1.dragStartBehavior, DragStartBehavior.down);
  });

  testWidgets('Switch on SwitchListTile changes mouse cursor when hovered', (
    WidgetTester tester,
  ) async {
    // Test SwitchListTile.adaptive() constructor
    await tester.pumpWidget(
      wrap(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return SwitchListTile.adaptive(
              mouseCursor: SystemMouseCursors.text,
              value: false,
              onChanged: (_) {},
            );
          },
        ),
      ),
    );
    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.byType(Switch)));

    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );

    // Test SwitchListTile() constructor
    await tester.pumpWidget(
      wrap(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return SwitchListTile(
              mouseCursor: SystemMouseCursors.forbidden,
              value: false,
              onChanged: (_) {},
            );
          },
        ),
      ),
    );

    await gesture.moveTo(tester.getCenter(find.byType(Switch)));
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.forbidden,
    );

    // Test default cursor
    await tester.pumpWidget(
      wrap(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return SwitchListTile(value: false, onChanged: (_) {});
          },
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
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return const SwitchListTile(value: false, onChanged: null);
          },
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );
  });

  testWidgets('Switch with splash radius set', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    const double splashRadius = 35;
    await tester.pumpWidget(
      wrap(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return SwitchListTile(splashRadius: splashRadius, value: false, onChanged: (_) {});
          },
        ),
      ),
    );

    await tester.pumpAndSettle();

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.moveTo(tester.getCenter(find.byType(Switch)));

    await tester.pumpAndSettle();
    expect(Material.of(tester.element(find.byType(Switch))), paints..circle(radius: splashRadius));
  });

  testWidgets(
    'The overlay color for the thumb of the switch resolves in active/pressed/hovered states',
    (WidgetTester tester) async {
      tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
      const Color activeThumbColor = Color(0xFF000000);
      const Color inactiveThumbColor = Color(0xFF000010);
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

      Widget buildSwitch({bool active = false, bool focused = false, bool useOverlay = true}) {
        return MaterialApp(
          home: Scaffold(
            body: SwitchListTile(
              value: active,
              onChanged: (_) {},
              thumbColor: MaterialStateProperty.resolveWith<Color>((Set<MaterialState> states) {
                if (states.contains(MaterialState.selected)) {
                  return activeThumbColor;
                }
                return inactiveThumbColor;
              }),
              overlayColor: useOverlay ? MaterialStateProperty.resolveWith(getOverlayColor) : null,
              hoverColor: hoverColor,
            ),
          ),
        );
      }

      // test inactive Switch, and overlayColor is set to null.
      await tester.pumpWidget(buildSwitch(useOverlay: false));
      await tester.press(find.byType(Switch));
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect()
          ..circle(color: inactiveThumbColor.withAlpha(kRadialReactionAlpha)),
        reason: 'Default inactive pressed Switch should have overlay color from thumbColor',
      );

      // test active Switch, and overlayColor is set to null.
      await tester.pumpWidget(buildSwitch(active: true, useOverlay: false));
      await tester.press(find.byType(Switch));
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect()
          ..circle(color: activeThumbColor.withAlpha(kRadialReactionAlpha)),
        reason: 'Default active pressed Switch should have overlay color from thumbColor',
      );

      // test inactive Switch with an overlayColor
      await tester.pumpWidget(buildSwitch());
      await tester.press(find.byType(Switch));
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect()
          ..circle(color: inactivePressedOverlayColor),
        reason: 'Inactive pressed Switch should have overlay color: $inactivePressedOverlayColor',
      );

      // test active Switch with an overlayColor
      await tester.pumpWidget(buildSwitch(active: true));
      await tester.press(find.byType(Switch));
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect()
          ..circle(color: activePressedOverlayColor),
        reason: 'Active pressed Switch should have overlay color: $activePressedOverlayColor',
      );

      await tester.pumpWidget(buildSwitch(focused: true));
      await tester.pumpAndSettle();

      // Start hovering
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      await gesture.moveTo(tester.getCenter(find.byType(Switch)));
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Switch))),
        paints
          ..rrect()
          ..circle(color: hoverOverlayColor),
        reason: 'Hovered Switch should use overlay color $hoverOverlayColor over $hoverColor',
      );
    },
  );

  testWidgets('SwitchListTile respects trackOutlineColor in active/enabled states', (
    WidgetTester tester,
  ) async {
    const Color activeEnabledTrackOutlineColor = Color(0xFF000001);
    const Color activeDisabledTrackOutlineColor = Color(0xFF000002);
    const Color inactiveEnabledTrackOutlineColor = Color(0xFF000003);
    const Color inactiveDisabledTrackOutlineColor = Color(0xFF000004);

    Color getOutlineColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        if (states.contains(MaterialState.selected)) {
          return activeDisabledTrackOutlineColor;
        }
        return inactiveDisabledTrackOutlineColor;
      }
      if (states.contains(MaterialState.selected)) {
        return activeEnabledTrackOutlineColor;
      }
      return inactiveEnabledTrackOutlineColor;
    }

    final MaterialStateProperty<Color> trackOutlineColor = MaterialStateColor.resolveWith(
      getOutlineColor,
    );

    Widget buildSwitchListTile({required bool enabled, required bool selected}) {
      return wrap(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return SwitchListTile(
              value: selected,
              trackOutlineColor: trackOutlineColor,
              onChanged: enabled ? (_) {} : null,
            );
          },
        ),
      );
    }

    await tester.pumpWidget(buildSwitchListTile(enabled: false, selected: false));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(style: PaintingStyle.fill)
        ..rrect(color: inactiveDisabledTrackOutlineColor, style: PaintingStyle.stroke),
    );

    await tester.pumpWidget(buildSwitchListTile(enabled: false, selected: true));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(style: PaintingStyle.fill)
        ..rrect(color: activeDisabledTrackOutlineColor, style: PaintingStyle.stroke),
    );

    await tester.pumpWidget(buildSwitchListTile(enabled: true, selected: false));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(style: PaintingStyle.fill)
        ..rrect(color: inactiveEnabledTrackOutlineColor, style: PaintingStyle.stroke),
    );

    await tester.pumpWidget(buildSwitchListTile(enabled: true, selected: true));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect(style: PaintingStyle.fill)
        ..rrect(color: activeEnabledTrackOutlineColor, style: PaintingStyle.stroke),
    );
  });

  testWidgets('SwitchListTile respects trackOutlineColor in hovered state', (
    WidgetTester tester,
  ) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    const Color hoveredTrackColor = Color(0xFF4caf50);

    Color getTrackOutlineColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.hovered)) {
        return hoveredTrackColor;
      }
      return Colors.transparent;
    }

    final MaterialStateProperty<Color> outlineColor = MaterialStateColor.resolveWith(
      getTrackOutlineColor,
    );

    Widget buildSwitchListTile() {
      return MaterialApp(
        theme: ThemeData(),
        home: wrap(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return SwitchListTile(
                value: false,
                trackOutlineColor: outlineColor,
                onChanged: (_) {},
              );
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(buildSwitchListTile());
    await tester.pumpAndSettle();

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.moveTo(tester.getCenter(find.byType(Switch)));

    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Switch))),
      paints
        ..rrect()
        ..rrect(color: hoveredTrackColor, style: PaintingStyle.stroke),
    );
  });

  testWidgets('SwitchListTile.control widget should not request focus on traversal', (
    WidgetTester tester,
  ) async {
    final GlobalKey firstChildKey = GlobalKey();
    final GlobalKey secondChildKey = GlobalKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Column(
            children: <Widget>[
              SwitchListTile(
                value: true,
                onChanged: (bool? value) {},
                title: Text('Hey', key: firstChildKey),
              ),
              SwitchListTile(
                value: true,
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

  testWidgets('SwitchListTile uses ListTileTheme controlAffinity', (WidgetTester tester) async {
    Widget buildView(ListTileControlAffinity controlAffinity) {
      return MaterialApp(
        home: Material(
          child: ListTileTheme(
            data: ListTileThemeData(controlAffinity: controlAffinity),
            child: SwitchListTile(
              value: true,
              title: const Text('SwitchListTile'),
              onChanged: (bool value) {},
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildView(ListTileControlAffinity.leading));
    final Finder leading = find.text('SwitchListTile');
    final Offset offsetLeading = tester.getTopLeft(leading);
    expect(offsetLeading, const Offset(92.0, 16.0));

    await tester.pumpWidget(buildView(ListTileControlAffinity.trailing));
    final Finder trailing = find.text('SwitchListTile');
    final Offset offsetTrailing = tester.getTopLeft(trailing);
    expect(offsetTrailing, const Offset(16.0, 16.0));

    await tester.pumpWidget(buildView(ListTileControlAffinity.platform));
    final Finder platform = find.text('SwitchListTile');
    final Offset offsetPlatform = tester.getTopLeft(platform);
    expect(offsetPlatform, const Offset(16.0, 16.0));
  });
}
