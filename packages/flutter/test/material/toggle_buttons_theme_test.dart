// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

Widget boilerplate({required Widget child}) {
  return Directionality(textDirection: TextDirection.ltr, child: Center(child: child));
}

void main() {
  test('ToggleButtonsThemeData copyWith, ==, hashCode basics', () {
    expect(const ToggleButtonsThemeData(), const ToggleButtonsThemeData().copyWith());
    expect(
      const ToggleButtonsThemeData().hashCode,
      const ToggleButtonsThemeData().copyWith().hashCode,
    );
  });

  test('ToggleButtonsThemeData lerp special cases', () {
    expect(ToggleButtonsThemeData.lerp(null, null, 0), null);
    const ToggleButtonsThemeData data = ToggleButtonsThemeData();
    expect(identical(ToggleButtonsThemeData.lerp(data, data, 0.5), data), true);
  });

  test('ToggleButtonsThemeData defaults', () {
    const ToggleButtonsThemeData themeData = ToggleButtonsThemeData();
    expect(themeData.textStyle, null);
    expect(themeData.constraints, null);
    expect(themeData.color, null);
    expect(themeData.selectedColor, null);
    expect(themeData.disabledColor, null);
    expect(themeData.fillColor, null);
    expect(themeData.focusColor, null);
    expect(themeData.highlightColor, null);
    expect(themeData.hoverColor, null);
    expect(themeData.splashColor, null);
    expect(themeData.borderColor, null);
    expect(themeData.selectedBorderColor, null);
    expect(themeData.disabledBorderColor, null);
    expect(themeData.borderRadius, null);
    expect(themeData.borderWidth, null);

    const ToggleButtonsTheme theme = ToggleButtonsTheme(
      data: ToggleButtonsThemeData(),
      child: SizedBox(),
    );
    expect(theme.data.textStyle, null);
    expect(theme.data.constraints, null);
    expect(theme.data.color, null);
    expect(theme.data.selectedColor, null);
    expect(theme.data.disabledColor, null);
    expect(theme.data.fillColor, null);
    expect(theme.data.focusColor, null);
    expect(theme.data.highlightColor, null);
    expect(theme.data.hoverColor, null);
    expect(theme.data.splashColor, null);
    expect(theme.data.borderColor, null);
    expect(theme.data.selectedBorderColor, null);
    expect(theme.data.disabledBorderColor, null);
    expect(theme.data.borderRadius, null);
    expect(theme.data.borderWidth, null);
  });

  testWidgets('Default ToggleButtonsThemeData debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const ToggleButtonsThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('ToggleButtonsThemeData implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const ToggleButtonsThemeData(
      textStyle: TextStyle(fontSize: 10),
      constraints: BoxConstraints(minHeight: 10.0, maxHeight: 20.0),
      color: Color(0xfffffff0),
      selectedColor: Color(0xfffffff1),
      disabledColor: Color(0xfffffff2),
      fillColor: Color(0xfffffff3),
      focusColor: Color(0xfffffff4),
      highlightColor: Color(0xfffffff5),
      hoverColor: Color(0xfffffff6),
      splashColor: Color(0xfffffff7),
      borderColor: Color(0xfffffff8),
      selectedBorderColor: Color(0xfffffff9),
      disabledBorderColor: Color(0xfffffffa),
      borderRadius: BorderRadius.all(Radius.circular(4.0)),
      borderWidth: 2.0,
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[
      'textStyle.inherit: true',
      'textStyle.size: 10.0',
      'constraints: BoxConstraints(0.0<=w<=Infinity, 10.0<=h<=20.0)',
      'color: ${const Color(0xfffffff0)}',
      'selectedColor: ${const Color(0xfffffff1)}',
      'disabledColor: ${const Color(0xfffffff2)}',
      'fillColor: ${const Color(0xfffffff3)}',
      'focusColor: ${const Color(0xfffffff4)}',
      'highlightColor: ${const Color(0xfffffff5)}',
      'hoverColor: ${const Color(0xfffffff6)}',
      'splashColor: ${const Color(0xfffffff7)}',
      'borderColor: ${const Color(0xfffffff8)}',
      'selectedBorderColor: ${const Color(0xfffffff9)}',
      'disabledBorderColor: ${const Color(0xfffffffa)}',
      'borderRadius: BorderRadius.circular(4.0)',
      'borderWidth: 2.0',
    ]);
  });

  testWidgets('Theme text style, except color, is applied', (WidgetTester tester) async {
    await tester.pumpWidget(
      Material(
        child: boilerplate(
          child: ToggleButtonsTheme(
            data: const ToggleButtonsThemeData(
              textStyle: TextStyle(
                color: Colors.orange,
                textBaseline: TextBaseline.ideographic,
                fontSize: 20.0,
              ),
            ),
            child: ToggleButtons(
              isSelected: const <bool>[false, true],
              onPressed: (int index) {},
              children: const <Widget>[Text('First child'), Text('Second child')],
            ),
          ),
        ),
      ),
    );

    TextStyle textStyle;
    textStyle =
        tester
            .widget<DefaultTextStyle>(
              find.descendant(
                of: find.widgetWithText(TextButton, 'First child'),
                matching: find.byType(DefaultTextStyle),
              ),
            )
            .style;
    expect(textStyle.textBaseline, TextBaseline.ideographic);
    expect(textStyle.fontSize, 20.0);
    expect(textStyle.color, isNot(Colors.orange));

    textStyle =
        tester
            .widget<DefaultTextStyle>(
              find.descendant(
                of: find.widgetWithText(TextButton, 'Second child'),
                matching: find.byType(DefaultTextStyle),
              ),
            )
            .style;
    expect(textStyle.textBaseline, TextBaseline.ideographic);
    expect(textStyle.fontSize, 20.0);
    expect(textStyle.color, isNot(Colors.orange));
  });

  testWidgets('Custom BoxConstraints', (WidgetTester tester) async {
    // Test for minimum constraints
    await tester.pumpWidget(
      Material(
        child: boilerplate(
          child: ToggleButtonsTheme(
            data: const ToggleButtonsThemeData(
              constraints: BoxConstraints(minWidth: 50.0, minHeight: 60.0),
            ),
            child: ToggleButtons(
              tapTargetSize: MaterialTapTargetSize.shrinkWrap,
              isSelected: const <bool>[false, false, false],
              onPressed: (int index) {},
              children: const <Widget>[
                Icon(Icons.check),
                Icon(Icons.access_alarm),
                Icon(Icons.cake),
              ],
            ),
          ),
        ),
      ),
    );

    Rect firstRect = tester.getRect(find.byType(TextButton).at(0));
    expect(firstRect.width, 50.0);
    expect(firstRect.height, 60.0);
    Rect secondRect = tester.getRect(find.byType(TextButton).at(1));
    expect(secondRect.width, 50.0);
    expect(secondRect.height, 60.0);
    Rect thirdRect = tester.getRect(find.byType(TextButton).at(2));
    expect(thirdRect.width, 50.0);
    expect(thirdRect.height, 60.0);

    // Test for maximum constraints
    await tester.pumpWidget(
      Material(
        child: boilerplate(
          child: ToggleButtonsTheme(
            data: const ToggleButtonsThemeData(
              constraints: BoxConstraints(maxWidth: 20.0, maxHeight: 10.0),
            ),
            child: ToggleButtons(
              tapTargetSize: MaterialTapTargetSize.shrinkWrap,
              isSelected: const <bool>[false, false, false],
              onPressed: (int index) {},
              children: const <Widget>[
                Icon(Icons.check),
                Icon(Icons.access_alarm),
                Icon(Icons.cake),
              ],
            ),
          ),
        ),
      ),
    );

    firstRect = tester.getRect(find.byType(TextButton).at(0));
    expect(firstRect.width, 20.0);
    expect(firstRect.height, 10.0);
    secondRect = tester.getRect(find.byType(TextButton).at(1));
    expect(secondRect.width, 20.0);
    expect(secondRect.height, 10.0);
    thirdRect = tester.getRect(find.byType(TextButton).at(2));
    expect(thirdRect.width, 20.0);
    expect(thirdRect.height, 10.0);
  });

  testWidgets('Theme text/icon colors for enabled, selected and disabled states', (
    WidgetTester tester,
  ) async {
    TextStyle buttonTextStyle(String text) {
      return tester
          .widget<DefaultTextStyle>(
            find.descendant(
              of: find.widgetWithText(TextButton, text),
              matching: find.byType(DefaultTextStyle),
            ),
          )
          .style;
    }

    IconTheme iconTheme(IconData icon) {
      return tester.widget(
        find.descendant(
          of: find.widgetWithIcon(TextButton, icon),
          matching: find.byType(IconTheme),
        ),
      );
    }

    final ThemeData theme = ThemeData();
    const Color enabledColor = Colors.lime;
    const Color selectedColor = Colors.green;
    const Color disabledColor = Colors.yellow;

    await tester.pumpWidget(
      Material(
        child: boilerplate(
          child: ToggleButtonsTheme(
            data: const ToggleButtonsThemeData(),
            child: ToggleButtons(
              color: enabledColor,
              isSelected: const <bool>[false],
              onPressed: (int index) {},
              children: const <Widget>[
                // This Row is used like this to test for both TextStyle
                // and IconTheme for Text and Icon widgets respectively.
                Row(children: <Widget>[Text('First child'), Icon(Icons.check)]),
              ],
            ),
          ),
        ),
      ),
    );
    // Custom theme enabled color
    expect(theme.colorScheme.onSurface, isNot(enabledColor));
    expect(buttonTextStyle('First child').color, enabledColor);
    expect(iconTheme(Icons.check).data.color, enabledColor);

    await tester.pumpWidget(
      Material(
        child: boilerplate(
          child: ToggleButtonsTheme(
            data: const ToggleButtonsThemeData(selectedColor: selectedColor),
            child: ToggleButtons(
              color: enabledColor,
              isSelected: const <bool>[true],
              onPressed: (int index) {},
              children: const <Widget>[
                Row(children: <Widget>[Text('First child'), Icon(Icons.check)]),
              ],
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    // Custom theme selected color
    expect(theme.colorScheme.primary, isNot(selectedColor));
    expect(buttonTextStyle('First child').color, selectedColor);
    expect(iconTheme(Icons.check).data.color, selectedColor);

    await tester.pumpWidget(
      Material(
        child: boilerplate(
          child: ToggleButtonsTheme(
            data: const ToggleButtonsThemeData(disabledColor: disabledColor),
            child: ToggleButtons(
              color: enabledColor,
              isSelected: const <bool>[false],
              children: const <Widget>[
                Row(children: <Widget>[Text('First child'), Icon(Icons.check)]),
              ],
            ),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    // Custom theme disabled color
    expect(theme.disabledColor, isNot(disabledColor));
    expect(buttonTextStyle('First child').color, disabledColor);
    expect(iconTheme(Icons.check).data.color, disabledColor);
  });

  testWidgets('Theme button fillColor', (WidgetTester tester) async {
    const Color customFillColor = Colors.green;
    await tester.pumpWidget(
      Material(
        child: boilerplate(
          child: ToggleButtonsTheme(
            data: const ToggleButtonsThemeData(fillColor: customFillColor),
            child: ToggleButtons(
              isSelected: const <bool>[true],
              onPressed: (int index) {},
              children: const <Widget>[
                Row(children: <Widget>[Text('First child')]),
              ],
            ),
          ),
        ),
      ),
    );

    final Material material = tester.widget<Material>(
      find.descendant(of: find.byType(TextButton), matching: find.byType(Material)),
    );
    expect(material.color, customFillColor);
    expect(material.type, MaterialType.button);
  });

  testWidgets('Custom Theme button fillColor in different states', (WidgetTester tester) async {
    Material buttonColor(String text) {
      return tester.widget<Material>(
        find.descendant(of: find.byType(TextButton), matching: find.widgetWithText(Material, text)),
      );
    }

    const Color enabledFillColor = Colors.green;
    const Color selectedFillColor = Colors.blue;
    const Color disabledFillColor = Colors.yellow;

    Color getColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        return selectedFillColor;
      } else if (states.contains(MaterialState.disabled)) {
        return disabledFillColor;
      }
      return enabledFillColor;
    }

    await tester.pumpWidget(
      Material(
        child: boilerplate(
          child: ToggleButtonsTheme(
            data: ToggleButtonsThemeData(fillColor: MaterialStateColor.resolveWith(getColor)),
            child: ToggleButtons(
              isSelected: const <bool>[true, false],
              onPressed: (int index) {},
              children: const <Widget>[Text('First child'), Text('Second child')],
            ),
          ),
        ),
      ),
    );

    await tester.pumpAndSettle();

    expect(buttonColor('First child').color, selectedFillColor);
    expect(buttonColor('Second child').color, enabledFillColor);

    await tester.pumpWidget(
      Material(
        child: boilerplate(
          child: ToggleButtonsTheme(
            data: ToggleButtonsThemeData(fillColor: MaterialStateColor.resolveWith(getColor)),
            child: ToggleButtons(
              isSelected: const <bool>[true, false],
              children: const <Widget>[Text('First child'), Text('Second child')],
            ),
          ),
        ),
      ),
    );

    expect(buttonColor('First child').color, disabledFillColor);
    expect(buttonColor('Second child').color, disabledFillColor);
  });

  testWidgets('Theme InkWell colors', (WidgetTester tester) async {
    const Color splashColor = Color(0xff4caf50);
    const Color highlightColor = Color(0xffcddc39);
    const Color hoverColor = Color(0xffffeb3b);
    const Color focusColor = Color(0xffffff00);
    final FocusNode focusNode = FocusNode();

    await tester.pumpWidget(
      Material(
        child: boilerplate(
          child: ToggleButtonsTheme(
            data: const ToggleButtonsThemeData(
              splashColor: splashColor,
              highlightColor: highlightColor,
              hoverColor: hoverColor,
              focusColor: focusColor,
            ),
            child: ToggleButtons(
              isSelected: const <bool>[true],
              onPressed: (int index) {},
              focusNodes: <FocusNode>[focusNode],
              children: const <Widget>[Text('First child')],
            ),
          ),
        ),
      ),
    );

    final Offset center = tester.getCenter(find.text('First child'));

    // splashColor
    // highlightColor
    final TestGesture touchGesture = await tester.createGesture();
    await touchGesture.down(center);
    await tester.pumpAndSettle();

    RenderObject inkFeatures;
    inkFeatures = tester.allRenderObjects.firstWhere((RenderObject object) {
      return object.runtimeType.toString() == '_RenderInkFeatures';
    });
    expect(inkFeatures, paints..circle(color: splashColor));

    await touchGesture.up();
    await tester.pumpAndSettle();

    // hoverColor
    final TestGesture hoverGesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await hoverGesture.addPointer();
    await hoverGesture.moveTo(center);
    await tester.pumpAndSettle();

    inkFeatures = tester.allRenderObjects.firstWhere((RenderObject object) {
      return object.runtimeType.toString() == '_RenderInkFeatures';
    });
    expect(inkFeatures, paints..rect(color: hoverColor));
    await hoverGesture.moveTo(Offset.zero);

    // focusColor
    FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    focusNode.requestFocus();
    await tester.pumpAndSettle();
    inkFeatures = tester.allRenderObjects.firstWhere((RenderObject object) {
      return object.runtimeType.toString() == '_RenderInkFeatures';
    });
    expect(inkFeatures, paints..rect(color: focusColor));

    await hoverGesture.removePointer();

    focusNode.dispose();
  });

  testWidgets('Theme border width and border colors for enabled, selected and disabled states', (
    WidgetTester tester,
  ) async {
    const Color borderColor = Color(0xff4caf50);
    const Color selectedBorderColor = Color(0xffcddc39);
    const Color disabledBorderColor = Color(0xffffeb3b);
    const double customWidth = 2.0;

    await tester.pumpWidget(
      Material(
        child: boilerplate(
          child: ToggleButtonsTheme(
            data: const ToggleButtonsThemeData(borderColor: borderColor, borderWidth: customWidth),
            child: ToggleButtons(
              isSelected: const <bool>[false],
              onPressed: (int index) {},
              children: const <Widget>[Text('First child')],
            ),
          ),
        ),
      ),
    );

    RenderObject toggleButtonRenderObject;
    toggleButtonRenderObject = tester.allRenderObjects.firstWhere((RenderObject object) {
      return object.runtimeType.toString() == '_SelectToggleButtonRenderObject';
    });
    expect(
      toggleButtonRenderObject,
      paints
        // physical model layer paint
        ..path()
        ..path(style: PaintingStyle.stroke, color: borderColor, strokeWidth: customWidth),
    );

    await tester.pumpWidget(
      Material(
        child: boilerplate(
          child: ToggleButtonsTheme(
            data: const ToggleButtonsThemeData(
              selectedBorderColor: selectedBorderColor,
              borderWidth: customWidth,
            ),
            child: ToggleButtons(
              isSelected: const <bool>[true],
              onPressed: (int index) {},
              children: const <Widget>[Text('First child')],
            ),
          ),
        ),
      ),
    );

    toggleButtonRenderObject = tester.allRenderObjects.firstWhere((RenderObject object) {
      return object.runtimeType.toString() == '_SelectToggleButtonRenderObject';
    });
    expect(
      toggleButtonRenderObject,
      paints
        // physical model layer paint
        ..path()
        ..path(style: PaintingStyle.stroke, color: selectedBorderColor, strokeWidth: customWidth),
    );

    await tester.pumpWidget(
      Material(
        child: boilerplate(
          child: ToggleButtonsTheme(
            data: const ToggleButtonsThemeData(
              disabledBorderColor: disabledBorderColor,
              borderWidth: customWidth,
            ),
            child: ToggleButtons(
              isSelected: const <bool>[false],
              children: const <Widget>[Text('First child')],
            ),
          ),
        ),
      ),
    );

    toggleButtonRenderObject = tester.allRenderObjects.firstWhere((RenderObject object) {
      return object.runtimeType.toString() == '_SelectToggleButtonRenderObject';
    });
    expect(
      toggleButtonRenderObject,
      paints
        // physical model layer paint
        ..path()
        ..path(style: PaintingStyle.stroke, color: disabledBorderColor, strokeWidth: customWidth),
    );
  });
}
