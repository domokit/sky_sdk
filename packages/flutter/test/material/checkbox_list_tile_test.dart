// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/feedback_tester.dart';

Widget wrap({required Widget child}) {
  return MediaQuery(
    data: const MediaQueryData(),
    child: Directionality(textDirection: TextDirection.ltr, child: Material(child: child)),
  );
}

void main() {
  testWidgets('CheckboxListTile control test', (WidgetTester tester) async {
    final List<dynamic> log = <dynamic>[];
    await tester.pumpWidget(
      wrap(
        child: CheckboxListTile(
          value: true,
          onChanged: (bool? value) {
            log.add(value);
          },
          title: const Text('Hello'),
        ),
      ),
    );
    await tester.tap(find.text('Hello'));
    log.add('-');
    await tester.tap(find.byType(Checkbox));
    expect(log, equals(<dynamic>[false, '-', false]));
  });

  testWidgets('Material2 - CheckboxListTile checkColor test', (WidgetTester tester) async {
    const Color checkBoxBorderColor = Color(0xff2196f3);
    Color checkBoxCheckColor = const Color(0xffFFFFFF);

    Widget buildFrame(Color? color) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: CheckboxListTile(value: true, checkColor: color, onChanged: (bool? value) {}),
        ),
      );
    }

    RenderBox getCheckboxListTileRenderer() {
      return tester.renderObject<RenderBox>(find.byType(CheckboxListTile));
    }

    await tester.pumpWidget(buildFrame(null));
    await tester.pumpAndSettle();
    expect(
      getCheckboxListTileRenderer(),
      paints
        ..path(color: checkBoxBorderColor)
        ..path(color: checkBoxCheckColor),
    );

    checkBoxCheckColor = const Color(0xFF000000);

    await tester.pumpWidget(buildFrame(checkBoxCheckColor));
    await tester.pumpAndSettle();
    expect(
      getCheckboxListTileRenderer(),
      paints
        ..path(color: checkBoxBorderColor)
        ..path(color: checkBoxCheckColor),
    );
  });

  testWidgets('Material3 - CheckboxListTile checkColor test', (WidgetTester tester) async {
    const Color checkBoxBorderColor = Color(0xff6750a4);
    Color checkBoxCheckColor = const Color(0xffFFFFFF);

    Widget buildFrame(Color? color) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: Material(
          child: CheckboxListTile(value: true, checkColor: color, onChanged: (bool? value) {}),
        ),
      );
    }

    RenderBox getCheckboxListTileRenderer() {
      return tester.renderObject<RenderBox>(find.byType(CheckboxListTile));
    }

    await tester.pumpWidget(buildFrame(null));
    await tester.pumpAndSettle();
    expect(
      getCheckboxListTileRenderer(),
      paints
        ..path(color: checkBoxBorderColor)
        ..path(color: checkBoxCheckColor),
    );

    checkBoxCheckColor = const Color(0xFF000000);

    await tester.pumpWidget(buildFrame(checkBoxCheckColor));
    await tester.pumpAndSettle();
    expect(
      getCheckboxListTileRenderer(),
      paints
        ..path(color: checkBoxBorderColor)
        ..path(color: checkBoxCheckColor),
    );
  });

  testWidgets('CheckboxListTile activeColor test', (WidgetTester tester) async {
    Widget buildFrame(Color? themeColor, Color? activeColor) {
      return wrap(
        child: Theme(
          data: ThemeData(
            checkboxTheme: CheckboxThemeData(
              fillColor: MaterialStateProperty.resolveWith<Color?>((Set<MaterialState> states) {
                return states.contains(MaterialState.selected) ? themeColor : null;
              }),
            ),
          ),
          child: CheckboxListTile(
            value: true,
            activeColor: activeColor,
            onChanged: (bool? value) {},
          ),
        ),
      );
    }

    RenderBox getCheckboxListTileRenderer() {
      return tester.renderObject<RenderBox>(find.byType(CheckboxListTile));
    }

    await tester.pumpWidget(buildFrame(const Color(0xFF000000), null));
    await tester.pumpAndSettle();
    expect(getCheckboxListTileRenderer(), paints..path(color: const Color(0xFF000000)));

    await tester.pumpWidget(buildFrame(const Color(0xFF000000), const Color(0xFFFFFFFF)));
    await tester.pumpAndSettle();
    expect(getCheckboxListTileRenderer(), paints..path(color: const Color(0xFFFFFFFF)));
  });

  testWidgets('CheckboxListTile can autofocus unless disabled.', (WidgetTester tester) async {
    final GlobalKey childKey = GlobalKey();

    await tester.pumpWidget(
      wrap(
        child: CheckboxListTile(
          value: true,
          onChanged: (_) {},
          title: Text('Hello', key: childKey),
          autofocus: true,
        ),
      ),
    );

    await tester.pump();
    expect(Focus.maybeOf(childKey.currentContext!)!.hasPrimaryFocus, isTrue);

    await tester.pumpWidget(
      wrap(
        child: CheckboxListTile(
          value: true,
          onChanged: null,
          title: Text('Hello', key: childKey),
          autofocus: true,
        ),
      ),
    );

    await tester.pump();
    expect(Focus.maybeOf(childKey.currentContext!)!.hasPrimaryFocus, isFalse);
  });

  testWidgets('CheckboxListTile contentPadding test', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        child: const Center(
          child: CheckboxListTile(
            value: false,
            onChanged: null,
            title: Text('Title'),
            contentPadding: EdgeInsets.fromLTRB(10, 18, 4, 2),
          ),
        ),
      ),
    );

    final Rect paddingRect = tester.getRect(find.byType(SafeArea));
    final Rect checkboxRect = tester.getRect(find.byType(Checkbox));
    final Rect titleRect = tester.getRect(find.text('Title'));

    final Rect tallerWidget = checkboxRect.height > titleRect.height ? checkboxRect : titleRect;

    // Check the offsets of Checkbox and title after padding is applied.
    expect(paddingRect.right, checkboxRect.right + 4);
    expect(paddingRect.left, titleRect.left - 10);

    // Calculate the remaining height from the default ListTile height.
    final double remainingHeight = 56 - tallerWidget.height;
    expect(paddingRect.top, tallerWidget.top - remainingHeight / 2 - 18);
    expect(paddingRect.bottom, tallerWidget.bottom + remainingHeight / 2 + 2);
  });

  testWidgets('CheckboxListTile tristate test', (WidgetTester tester) async {
    bool? value = false;
    bool tristate = false;

    await tester.pumpWidget(
      Material(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return wrap(
              child: CheckboxListTile(
                title: const Text('Title'),
                tristate: tristate,
                value: value,
                onChanged: (bool? v) {
                  setState(() {
                    value = v;
                  });
                },
              ),
            );
          },
        ),
      ),
    );

    expect(tester.widget<Checkbox>(find.byType(Checkbox)).value, false);

    // Tap the checkbox when tristate is disabled.
    await tester.tap(find.byType(Checkbox));
    await tester.pumpAndSettle();
    expect(value, true);

    await tester.tap(find.byType(Checkbox));
    await tester.pumpAndSettle();
    expect(value, false);

    // Tap the listTile when tristate is disabled.
    await tester.tap(find.byType(ListTile));
    await tester.pumpAndSettle();
    expect(value, true);

    await tester.tap(find.byType(ListTile));
    await tester.pumpAndSettle();
    expect(value, false);

    // Enable tristate
    tristate = true;
    await tester.pumpAndSettle();

    expect(tester.widget<Checkbox>(find.byType(Checkbox)).value, false);

    // Tap the checkbox when tristate is enabled.
    await tester.tap(find.byType(Checkbox));
    await tester.pumpAndSettle();
    expect(value, true);

    await tester.tap(find.byType(Checkbox));
    await tester.pumpAndSettle();
    expect(value, null);

    await tester.tap(find.byType(Checkbox));
    await tester.pumpAndSettle();
    expect(value, false);

    // Tap the listTile when tristate is enabled.
    await tester.tap(find.byType(ListTile));
    await tester.pumpAndSettle();
    expect(value, true);

    await tester.tap(find.byType(ListTile));
    await tester.pumpAndSettle();
    expect(value, null);

    await tester.tap(find.byType(ListTile));
    await tester.pumpAndSettle();
    expect(value, false);
  });

  testWidgets('CheckboxListTile respects shape', (WidgetTester tester) async {
    const ShapeBorder shapeBorder = RoundedRectangleBorder(
      borderRadius: BorderRadius.horizontal(right: Radius.circular(100)),
    );

    await tester.pumpWidget(
      wrap(
        child: const CheckboxListTile(
          value: false,
          onChanged: null,
          title: Text('Title'),
          shape: shapeBorder,
        ),
      ),
    );

    expect(tester.widget<InkWell>(find.byType(InkWell)).customBorder, shapeBorder);
  });

  testWidgets('CheckboxListTile respects tileColor', (WidgetTester tester) async {
    final Color tileColor = Colors.red.shade500;

    await tester.pumpWidget(
      wrap(
        child: Center(
          child: CheckboxListTile(
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

  testWidgets('CheckboxListTile respects selectedTileColor', (WidgetTester tester) async {
    final Color selectedTileColor = Colors.green.shade500;

    await tester.pumpWidget(
      wrap(
        child: Center(
          child: CheckboxListTile(
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

  testWidgets('CheckboxListTile selected item text Color', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/pull/76908

    const Color activeColor = Color(0xff00ff00);

    Widget buildFrame({Color? activeColor, Color? fillColor}) {
      return MaterialApp(
        theme: ThemeData.light().copyWith(
          checkboxTheme: CheckboxThemeData(
            fillColor: MaterialStateProperty.resolveWith<Color?>((Set<MaterialState> states) {
              return states.contains(MaterialState.selected) ? fillColor : null;
            }),
          ),
        ),
        home: Scaffold(
          body: Center(
            child: CheckboxListTile(
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

    await tester.pumpWidget(buildFrame(fillColor: activeColor));
    expect(textColor('title'), activeColor);

    await tester.pumpWidget(buildFrame(activeColor: activeColor));
    expect(textColor('title'), activeColor);
  });

  testWidgets('CheckboxListTile respects checkbox shape and side', (WidgetTester tester) async {
    Widget buildApp(BorderSide side, OutlinedBorder shape) {
      return MaterialApp(
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return CheckboxListTile(
                  value: false,
                  onChanged: (bool? newValue) {},
                  side: side,
                  checkboxShape: shape,
                );
              },
            ),
          ),
        ),
      );
    }

    const RoundedRectangleBorder border1 = RoundedRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(5)),
    );
    const BorderSide side1 = BorderSide(color: Color(0xfff44336));
    await tester.pumpWidget(buildApp(side1, border1));
    expect(tester.widget<CheckboxListTile>(find.byType(CheckboxListTile)).side, side1);
    expect(tester.widget<CheckboxListTile>(find.byType(CheckboxListTile)).checkboxShape, border1);
    expect(tester.widget<Checkbox>(find.byType(Checkbox)).side, side1);
    expect(tester.widget<Checkbox>(find.byType(Checkbox)).shape, border1);
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints..drrect(
        color: const Color(0xfff44336),
        outer: RRect.fromLTRBR(11.0, 11.0, 29.0, 29.0, const Radius.circular(5)),
        inner: RRect.fromLTRBR(12.0, 12.0, 28.0, 28.0, const Radius.circular(4)),
      ),
    );
    const RoundedRectangleBorder border2 = RoundedRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(5)),
    );
    const BorderSide side2 = BorderSide(width: 4.0, color: Color(0xff424242));
    await tester.pumpWidget(buildApp(side2, border2));
    expect(tester.widget<Checkbox>(find.byType(Checkbox)).side, side2);
    expect(tester.widget<Checkbox>(find.byType(Checkbox)).shape, border2);
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints..drrect(
        color: const Color(0xff424242),
        outer: RRect.fromLTRBR(11.0, 11.0, 29.0, 29.0, const Radius.circular(5)),
        inner: RRect.fromLTRBR(15.0, 15.0, 25.0, 25.0, const Radius.circular(1)),
      ),
    );
  });

  testWidgets('CheckboxListTile respects visualDensity', (WidgetTester tester) async {
    const Key key = Key('test');
    Future<void> buildTest(VisualDensity visualDensity) async {
      return tester.pumpWidget(
        wrap(
          child: Center(
            child: CheckboxListTile(
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

  testWidgets('CheckboxListTile respects focusNode', (WidgetTester tester) async {
    final GlobalKey childKey = GlobalKey();
    await tester.pumpWidget(
      wrap(
        child: Center(
          child: CheckboxListTile(
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

  testWidgets('CheckboxListTile onFocusChange callback', (WidgetTester tester) async {
    final FocusNode node = FocusNode(debugLabel: 'CheckboxListTile onFocusChange');
    bool gotFocus = false;
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: CheckboxListTile(
            value: true,
            focusNode: node,
            onFocusChange: (bool focused) {
              gotFocus = focused;
            },
            onChanged: (bool? value) {},
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

  testWidgets('CheckboxListTile can be disabled', (WidgetTester tester) async {
    bool? value = false;
    bool enabled = true;

    await tester.pumpWidget(
      Material(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return wrap(
              child: CheckboxListTile(
                title: const Text('Title'),
                enabled: enabled,
                value: value,
                onChanged: (bool? v) {
                  setState(() {
                    value = v;
                    enabled = !enabled;
                  });
                },
              ),
            );
          },
        ),
      ),
    );

    final Finder checkbox = find.byType(Checkbox);
    // verify initial values
    expect(tester.widget<Checkbox>(checkbox).value, false);
    expect(enabled, true);

    // Tap the checkbox to disable CheckboxListTile
    await tester.tap(checkbox);
    await tester.pumpAndSettle();
    expect(tester.widget<Checkbox>(checkbox).value, true);
    expect(enabled, false);
    await tester.tap(checkbox);
    await tester.pumpAndSettle();
    expect(tester.widget<Checkbox>(checkbox).value, true);
  });

  testWidgets('CheckboxListTile respects mouseCursor when hovered', (WidgetTester tester) async {
    // Test Checkbox() constructor
    await tester.pumpWidget(
      wrap(
        child: MouseRegion(
          cursor: SystemMouseCursors.forbidden,
          child: CheckboxListTile(
            mouseCursor: SystemMouseCursors.text,
            value: true,
            onChanged: (_) {},
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.byType(Checkbox)));

    await tester.pump();

    await gesture.moveTo(tester.getCenter(find.byType(Checkbox)));
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );

    // Test default cursor
    await tester.pumpWidget(wrap(child: CheckboxListTile(value: true, onChanged: (_) {})));

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );

    // Test default cursor when disabled
    await tester.pumpWidget(
      wrap(
        child: const MouseRegion(
          cursor: SystemMouseCursors.forbidden,
          child: CheckboxListTile(value: true, onChanged: null),
        ),
      ),
    );

    await gesture.moveTo(tester.getCenter(find.byType(Checkbox)));
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );

    // Test cursor when tristate
    await tester.pumpWidget(
      wrap(
        child: const MouseRegion(
          cursor: SystemMouseCursors.forbidden,
          child: CheckboxListTile(
            value: null,
            tristate: true,
            onChanged: null,
            mouseCursor: _SelectedGrabMouseCursor(),
          ),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.grab,
    );

    await tester.pumpAndSettle();
  });

  testWidgets('CheckboxListTile respects fillColor in enabled/disabled states', (
    WidgetTester tester,
  ) async {
    const Color activeEnabledFillColor = Color(0xFF000001);
    const Color activeDisabledFillColor = Color(0xFF000002);

    Color getFillColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return activeDisabledFillColor;
      }
      return activeEnabledFillColor;
    }

    final MaterialStateProperty<Color> fillColor = MaterialStateColor.resolveWith(getFillColor);

    Widget buildFrame({required bool enabled}) {
      return wrap(
        child: CheckboxListTile(
          value: true,
          fillColor: fillColor,
          onChanged: enabled ? (bool? value) {} : null,
        ),
      );
    }

    RenderBox getCheckboxRenderer() {
      return tester.renderObject<RenderBox>(find.byType(Checkbox));
    }

    await tester.pumpWidget(buildFrame(enabled: true));
    await tester.pumpAndSettle();
    expect(getCheckboxRenderer(), paints..path(color: activeEnabledFillColor));

    await tester.pumpWidget(buildFrame(enabled: false));
    await tester.pumpAndSettle();
    expect(getCheckboxRenderer(), paints..path(color: activeDisabledFillColor));
  });

  testWidgets('CheckboxListTile respects fillColor in hovered state', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    const Color hoveredFillColor = Color(0xFF000001);

    Color getFillColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.hovered)) {
        return hoveredFillColor;
      }
      return Colors.transparent;
    }

    final MaterialStateProperty<Color> fillColor = MaterialStateColor.resolveWith(getFillColor);

    Widget buildFrame() {
      return wrap(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return CheckboxListTile(value: true, fillColor: fillColor, onChanged: (bool? value) {});
          },
        ),
      );
    }

    RenderBox getCheckboxRenderer() {
      return tester.renderObject<RenderBox>(find.byType(Checkbox));
    }

    await tester.pumpWidget(buildFrame());
    await tester.pumpAndSettle();

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byType(Checkbox)));
    await tester.pumpAndSettle();

    expect(getCheckboxRenderer(), paints..path(color: hoveredFillColor));
  });

  testWidgets('CheckboxListTile respects hoverColor', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    bool? value = true;
    Widget buildApp({bool enabled = true}) {
      return wrap(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return CheckboxListTile(
              value: value,
              onChanged:
                  enabled
                      ? (bool? newValue) {
                        setState(() {
                          value = newValue;
                        });
                      }
                      : null,
              hoverColor: Colors.orange[500],
            );
          },
        ),
      );
    }

    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints
        ..path(style: PaintingStyle.fill)
        ..path(style: PaintingStyle.stroke, strokeWidth: 2.0),
    );

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.moveTo(tester.getCenter(find.byType(Checkbox)));

    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints
        ..circle(color: Colors.orange[500])
        ..path(style: PaintingStyle.fill)
        ..path(style: PaintingStyle.stroke, strokeWidth: 2.0),
    );

    // Check what happens when disabled.
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints
        ..path(style: PaintingStyle.fill)
        ..path(style: PaintingStyle.stroke, strokeWidth: 2.0),
    );
  });

  testWidgets(
    'Material2 - CheckboxListTile respects overlayColor in active/pressed/hovered states',
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

      const double splashRadius = 24.0;

      Widget buildCheckbox({bool active = false, bool useOverlay = true}) {
        return MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(
            child: CheckboxListTile(
              value: active,
              onChanged: (_) {},
              fillColor: const MaterialStatePropertyAll<Color>(fillColor),
              overlayColor: useOverlay ? MaterialStateProperty.resolveWith(getOverlayColor) : null,
              hoverColor: hoverColor,
              splashRadius: splashRadius,
            ),
          ),
        );
      }

      await tester.pumpWidget(buildCheckbox(useOverlay: false));
      final TestGesture gesture1 = await tester.startGesture(
        tester.getCenter(find.byType(Checkbox)),
      );
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Checkbox))),
        paints
          ..circle()
          ..circle(color: fillColor.withAlpha(kRadialReactionAlpha), radius: splashRadius),
        reason: 'Default inactive pressed Checkbox should have overlay color from fillColor',
      );

      await tester.pumpWidget(buildCheckbox(active: true, useOverlay: false));
      final TestGesture gesture2 = await tester.startGesture(
        tester.getCenter(find.byType(Checkbox)),
      );
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Checkbox))),
        paints
          ..circle()
          ..circle(color: fillColor.withAlpha(kRadialReactionAlpha), radius: splashRadius),
        reason: 'Default active pressed Checkbox should have overlay color from fillColor',
      );

      await tester.pumpWidget(buildCheckbox());
      final TestGesture gesture3 = await tester.startGesture(
        tester.getCenter(find.byType(Checkbox)),
      );
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Checkbox))),
        paints
          ..circle()
          ..circle(color: inactivePressedOverlayColor, radius: splashRadius),
        reason: 'Inactive pressed Checkbox should have overlay color: $inactivePressedOverlayColor',
      );

      await tester.pumpWidget(buildCheckbox(active: true));
      final TestGesture gesture4 = await tester.startGesture(
        tester.getCenter(find.byType(Checkbox)),
      );
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Checkbox))),
        paints
          ..circle()
          ..circle(color: activePressedOverlayColor, radius: splashRadius),
        reason: 'Active pressed Checkbox should have overlay color: $activePressedOverlayColor',
      );

      // Start hovering
      await tester.pumpWidget(Container());
      await tester.pumpWidget(buildCheckbox());
      await tester.pumpAndSettle();

      final TestGesture gesture5 = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture5.addPointer();
      await gesture5.moveTo(tester.getCenter(find.byType(Checkbox)));
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Checkbox))),
        paints..circle(color: hoverOverlayColor, radius: splashRadius),
        reason: 'Hovered Checkbox should use overlay color $hoverOverlayColor over $hoverColor',
      );

      // Finish gestures to release resources.
      await gesture1.up();
      await gesture2.up();
      await gesture3.up();
      await gesture4.up();
      await tester.pumpAndSettle();
    },
  );

  testWidgets(
    'Material3 - CheckboxListTile respects overlayColor in active/pressed/hovered states',
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

      const double splashRadius = 24.0;

      Widget buildCheckbox({bool active = false, bool useOverlay = true}) {
        return MaterialApp(
          theme: ThemeData(useMaterial3: true),
          home: Material(
            child: CheckboxListTile(
              value: active,
              onChanged: (_) {},
              fillColor: const MaterialStatePropertyAll<Color>(fillColor),
              overlayColor: useOverlay ? MaterialStateProperty.resolveWith(getOverlayColor) : null,
              hoverColor: hoverColor,
              splashRadius: splashRadius,
            ),
          ),
        );
      }

      await tester.pumpWidget(buildCheckbox(useOverlay: false));
      final TestGesture gesture1 = await tester.startGesture(
        tester.getCenter(find.byType(Checkbox)),
      );
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Checkbox))),
        kIsWeb
            ? (paints
              ..circle()
              ..circle(color: fillColor.withAlpha(kRadialReactionAlpha), radius: splashRadius))
            : (paints
              ..circle(color: fillColor.withAlpha(kRadialReactionAlpha), radius: splashRadius)),
        reason: 'Default inactive pressed Checkbox should have overlay color from fillColor',
      );

      await tester.pumpWidget(buildCheckbox(active: true, useOverlay: false));
      final TestGesture gesture2 = await tester.startGesture(
        tester.getCenter(find.byType(Checkbox)),
      );
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Checkbox))),
        kIsWeb
            ? (paints
              ..circle()
              ..circle(color: fillColor.withAlpha(kRadialReactionAlpha), radius: splashRadius))
            : (paints
              ..circle(color: fillColor.withAlpha(kRadialReactionAlpha), radius: splashRadius)),
        reason: 'Default active pressed Checkbox should have overlay color from fillColor',
      );

      await tester.pumpWidget(buildCheckbox());
      final TestGesture gesture3 = await tester.startGesture(
        tester.getCenter(find.byType(Checkbox)),
      );
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Checkbox))),
        kIsWeb
            ? (paints
              ..circle()
              ..circle(color: inactivePressedOverlayColor, radius: splashRadius))
            : (paints..circle(color: inactivePressedOverlayColor, radius: splashRadius)),
        reason: 'Inactive pressed Checkbox should have overlay color: $inactivePressedOverlayColor',
      );

      await tester.pumpWidget(buildCheckbox(active: true));
      final TestGesture gesture4 = await tester.startGesture(
        tester.getCenter(find.byType(Checkbox)),
      );
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Checkbox))),
        kIsWeb
            ? (paints
              ..circle()
              ..circle(color: activePressedOverlayColor, radius: splashRadius))
            : (paints..circle(color: activePressedOverlayColor, radius: splashRadius)),
        reason: 'Active pressed Checkbox should have overlay color: $activePressedOverlayColor',
      );

      // Start hovering
      await tester.pumpWidget(Container());
      await tester.pumpWidget(buildCheckbox());
      await tester.pumpAndSettle();

      final TestGesture gesture5 = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture5.addPointer();
      await gesture5.moveTo(tester.getCenter(find.byType(Checkbox)));
      await tester.pumpAndSettle();

      expect(
        Material.of(tester.element(find.byType(Checkbox))),
        paints..circle(color: hoverOverlayColor, radius: splashRadius),
        reason: 'Hovered Checkbox should use overlay color $hoverOverlayColor over $hoverColor',
      );

      // Finish gestures to release resources.
      await gesture1.up();
      await gesture2.up();
      await gesture3.up();
      await gesture4.up();
      await tester.pumpAndSettle();
    },
  );

  testWidgets('CheckboxListTile respects splashRadius', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    const double splashRadius = 30;
    Widget buildApp() {
      return wrap(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return CheckboxListTile(
              value: false,
              onChanged: (bool? newValue) {},
              hoverColor: Colors.orange[500],
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
    await gesture.moveTo(tester.getCenter(find.byType(Checkbox)));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints..circle(color: Colors.orange[500], radius: splashRadius),
    );
  });

  testWidgets('CheckboxListTile respects materialTapTargetSize', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(child: CheckboxListTile(value: true, onChanged: (bool? newValue) {})),
    );

    // default test
    expect(tester.getSize(find.byType(Checkbox)), const Size(40.0, 40.0));

    await tester.pumpWidget(
      wrap(
        child: CheckboxListTile(
          materialTapTargetSize: MaterialTapTargetSize.padded,
          value: true,
          onChanged: (bool? newValue) {},
        ),
      ),
    );

    expect(tester.getSize(find.byType(Checkbox)), const Size(48.0, 48.0));
  });

  testWidgets('Material3 - CheckboxListTile respects isError', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData(useMaterial3: true);
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    bool? value = true;
    Widget buildApp() {
      return MaterialApp(
        theme: themeData,
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return CheckboxListTile(
                  isError: true,
                  value: value,
                  onChanged: (bool? newValue) {
                    setState(() {
                      value = newValue;
                    });
                  },
                );
              },
            ),
          ),
        ),
      );
    }

    // Default color
    await tester.pumpWidget(Container());
    await tester.pumpWidget(buildApp());
    await tester.pumpAndSettle();
    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints
        ..path(color: themeData.colorScheme.error)
        ..path(color: themeData.colorScheme.onError),
    );

    // Start hovering
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byType(Checkbox)));
    await tester.pumpAndSettle();

    expect(
      Material.of(tester.element(find.byType(Checkbox))),
      paints
        ..circle(color: themeData.colorScheme.error.withOpacity(0.08))
        ..path(color: themeData.colorScheme.error),
    );
  });

  testWidgets('CheckboxListTile.adaptive shows the correct checkbox platform widget', (
    WidgetTester tester,
  ) async {
    Widget buildApp(TargetPlatform platform) {
      return MaterialApp(
        theme: ThemeData(platform: platform),
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return CheckboxListTile.adaptive(value: false, onChanged: (bool? newValue) {});
              },
            ),
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

      expect(find.byType(CupertinoCheckbox), findsOneWidget);
    }

    for (final TargetPlatform platform in <TargetPlatform>[
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.linux,
      TargetPlatform.windows,
    ]) {
      await tester.pumpWidget(buildApp(platform));
      await tester.pumpAndSettle();

      expect(find.byType(CupertinoCheckbox), findsNothing);
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

    testWidgets('CheckboxListTile respects enableFeedback', (WidgetTester tester) async {
      Future<void> buildTest(bool enableFeedback) async {
        return tester.pumpWidget(
          wrap(
            child: Center(
              child: CheckboxListTile(
                value: false,
                onChanged: (bool? value) {},
                enableFeedback: enableFeedback,
              ),
            ),
          ),
        );
      }

      await buildTest(false);
      await tester.tap(find.byType(CheckboxListTile));
      await tester.pump(const Duration(seconds: 1));
      expect(feedback.clickSoundCount, 0);
      expect(feedback.hapticCount, 0);

      await buildTest(true);
      await tester.tap(find.byType(CheckboxListTile));
      await tester.pump(const Duration(seconds: 1));
      expect(feedback.clickSoundCount, 1);
      expect(feedback.hapticCount, 0);
    });
  });

  testWidgets('CheckboxListTile has proper semantics', (WidgetTester tester) async {
    final List<dynamic> log = <dynamic>[];
    final SemanticsHandle handle = tester.ensureSemantics();
    await tester.pumpWidget(
      wrap(
        child: CheckboxListTile(
          value: true,
          onChanged: (bool? value) {
            log.add(value);
          },
          title: const Text('Hello'),
          checkboxSemanticLabel: 'there',
          internalAddSemanticForOnTap: true,
        ),
      ),
    );

    expect(
      tester.getSemantics(find.byType(CheckboxListTile)),
      matchesSemantics(
        isButton: true,
        hasCheckedState: true,
        isChecked: true,
        hasEnabledState: true,
        isEnabled: true,
        hasTapAction: true,
        hasFocusAction: true,
        isFocusable: true,
        label: 'Hello\nthere',
      ),
    );

    handle.dispose();
  });

  testWidgets('CheckboxListTile.control widget should not request focus on traversal', (
    WidgetTester tester,
  ) async {
    final GlobalKey firstChildKey = GlobalKey();
    final GlobalKey secondChildKey = GlobalKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Column(
            children: <Widget>[
              CheckboxListTile(
                value: true,
                onChanged: (bool? value) {},
                title: Text('Hey', key: firstChildKey),
              ),
              CheckboxListTile(
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

  testWidgets('CheckboxListTile uses ListTileTheme controlAffinity', (WidgetTester tester) async {
    Widget buildListTile(ListTileControlAffinity controlAffinity) {
      return MaterialApp(
        home: Material(
          child: ListTileTheme(
            data: ListTileThemeData(controlAffinity: controlAffinity),
            child: CheckboxListTile(value: false, onChanged: (bool? value) {}),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildListTile(ListTileControlAffinity.trailing));
    final Finder trailing = find.byType(Checkbox);
    final Offset offsetTrailing = tester.getTopLeft(trailing);
    expect(offsetTrailing, const Offset(736.0, 8.0));

    await tester.pumpWidget(buildListTile(ListTileControlAffinity.leading));
    final Finder leading = find.byType(Checkbox);
    final Offset offsetLeading = tester.getTopLeft(leading);
    expect(offsetLeading, const Offset(16.0, 8.0));

    await tester.pumpWidget(buildListTile(ListTileControlAffinity.platform));
    final Finder platform = find.byType(Checkbox);
    final Offset offsetPlatform = tester.getTopLeft(platform);
    expect(offsetPlatform, const Offset(736.0, 8.0));
  });

  testWidgets('CheckboxListTile renders with default scale', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: CheckboxListTile(value: false, onChanged: null))),
    );

    final Finder transformFinder = find.ancestor(
      of: find.byType(Checkbox),
      matching: find.byType(Transform),
    );

    expect(transformFinder, findsNothing);
  });

  testWidgets('CheckboxListTile respects checkboxScaleFactor', (WidgetTester tester) async {
    const double scale = 1.5;

    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: CheckboxListTile(value: false, onChanged: null, checkboxScaleFactor: scale),
        ),
      ),
    );

    final Transform widget = tester.widget(
      find.ancestor(of: find.byType(Checkbox), matching: find.byType(Transform)),
    );

    expect(widget.transform.getMaxScaleOnAxis(), scale);
  });
}

class _SelectedGrabMouseCursor extends MaterialStateMouseCursor {
  const _SelectedGrabMouseCursor();

  @override
  MouseCursor resolve(Set<MaterialState> states) {
    if (states.contains(MaterialState.selected)) {
      return SystemMouseCursors.grab;
    }
    return SystemMouseCursors.basic;
  }

  @override
  String get debugDescription => '_SelectedGrabMouseCursor()';
}
