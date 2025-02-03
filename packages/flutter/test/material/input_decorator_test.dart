// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'dart:async';
import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

const Duration kTransitionDuration = Duration(milliseconds: 167);

const String hintText = 'hint';
const String inputText = 'text';
const String labelText = 'label';
const String errorText = 'error';
const String helperText = 'helper';
const String counterText = 'counter';

const Key customLabelKey = Key('label');
const Widget customLabel = Text.rich(
  key: customLabelKey,
  TextSpan(
    children: <InlineSpan>[
      TextSpan(text: 'label'),
      WidgetSpan(child: Text('*', style: TextStyle(color: Colors.red))),
    ],
  ),
);

const String twoLines = 'line1\nline2';
const String threeLines = 'line1\nline2\nline3';

Widget buildInputDecorator({
  InputDecoration decoration = const InputDecoration(),
  ThemeData? theme,
  InputDecorationTheme? inputDecorationTheme,
  IconButtonThemeData? iconButtonTheme,
  TextDirection textDirection = TextDirection.ltr,
  bool expands = false,
  bool isEmpty = false,
  bool isFocused = false,
  bool isHovering = false,
  bool useIntrinsicWidth = false,
  TextStyle? baseStyle,
  TextAlignVertical? textAlignVertical,
  VisualDensity? visualDensity,
  Widget child = const Text(
    inputText,
    // Use a text style compliant with M3 specification (which is bodyLarge for text fields).
    style: TextStyle(fontSize: 16.0, fontWeight: FontWeight.w400, letterSpacing: 0.5, height: 1.50),
  ),
}) {
  Widget widget = InputDecorator(
    expands: expands,
    decoration: decoration,
    isEmpty: isEmpty,
    isFocused: isFocused,
    isHovering: isHovering,
    baseStyle: baseStyle,
    textAlignVertical: textAlignVertical,
    child: child,
  );

  if (useIntrinsicWidth) {
    widget = IntrinsicWidth(child: widget);
  }

  return MaterialApp(
    home: Material(
      child: Builder(
        builder: (BuildContext context) {
          return Theme(
            data: (theme ?? Theme.of(context)).copyWith(
              inputDecorationTheme: inputDecorationTheme,
              iconButtonTheme: iconButtonTheme,
              visualDensity: visualDensity,
            ),
            child: Align(
              alignment: Alignment.topLeft,
              child: Directionality(textDirection: textDirection, child: widget),
            ),
          );
        },
      ),
    ),
  );
}

Finder findBorderPainter() {
  return find.descendant(
    of: find.byWidgetPredicate((Widget w) => '${w.runtimeType}' == '_BorderContainer'),
    matching: find.byWidgetPredicate((Widget w) => w is CustomPaint),
  );
}

double getBorderBottom(WidgetTester tester) {
  final RenderBox box = InputDecorator.containerOf(tester.element(findBorderPainter()))!;
  return box.size.height;
}

Finder findLabel() {
  return find.descendant(
    of: find.byType(MatrixTransition),
    matching: find.byWidgetPredicate((Widget w) => w is Text),
  );
}

Rect getLabelRect(WidgetTester tester) {
  return tester.getRect(findLabel());
}

Offset getLabelCenter(WidgetTester tester) {
  return getLabelRect(tester).center;
}

TextStyle getLabelStyle(WidgetTester tester) {
  return tester
      .firstWidget<AnimatedDefaultTextStyle>(
        find.ancestor(of: findLabel(), matching: find.byType(AnimatedDefaultTextStyle)),
      )
      .style;
}

Finder findCustomLabel() {
  return find.byKey(customLabelKey);
}

Rect getCustomLabelRect(WidgetTester tester) {
  return tester.getRect(findCustomLabel());
}

Offset getCustomLabelCenter(WidgetTester tester) {
  return getCustomLabelRect(tester).center;
}

Finder findInputText() {
  return find.text(inputText);
}

Rect getInputRect(WidgetTester tester) {
  return tester.getRect(findInputText());
}

Offset getInputCenter(WidgetTester tester) {
  return getInputRect(tester).center;
}

Finder findHint() {
  return find.text(hintText);
}

Rect getHintRect(WidgetTester tester) {
  return tester.getRect(findHint());
}

Offset getHintCenter(WidgetTester tester) {
  return getHintRect(tester).center;
}

double getHintOpacity(WidgetTester tester) {
  return getOpacity(tester, hintText);
}

Finder findHelper() {
  return find.text(helperText);
}

TextStyle getHintStyle(WidgetTester tester) {
  return tester
      .widget<RichText>(find.descendant(of: findHint(), matching: find.byType(RichText)))
      .text
      .style!;
}

Rect getHelperRect(WidgetTester tester) {
  return tester.getRect(findHelper());
}

TextStyle getHelperStyle(WidgetTester tester) {
  return tester
      .widget<RichText>(find.descendant(of: findHelper(), matching: find.byType(RichText)))
      .text
      .style!;
}

Finder findError() {
  return find.text(errorText);
}

Rect getErrorRect(WidgetTester tester) {
  return tester.getRect(findError());
}

TextStyle getErrorStyle(WidgetTester tester) {
  return tester
      .widget<RichText>(find.descendant(of: findError(), matching: find.byType(RichText)))
      .text
      .style!;
}

Finder findCounter() {
  return find.text(counterText);
}

Rect getCounterRect(WidgetTester tester) {
  return tester.getRect(findCounter());
}

TextStyle getCounterStyle(WidgetTester tester) {
  return tester
      .widget<RichText>(find.descendant(of: findCounter(), matching: find.byType(RichText)))
      .text
      .style!;
}

Finder findDecorator() {
  return find.byType(InputDecorator);
}

Rect getDecoratorRect(WidgetTester tester) {
  return tester.getRect(findDecorator());
}

Offset getDecoratorCenter(WidgetTester tester) {
  return getDecoratorRect(tester).center;
}

Rect getContainerRect(WidgetTester tester) {
  final RenderBox box = InputDecorator.containerOf(tester.element(findBorderPainter()))!;
  return box.paintBounds;
}

InputBorder? getBorder(WidgetTester tester) {
  if (!tester.any(findBorderPainter())) {
    return null;
  }
  final CustomPaint customPaint = tester.widget(findBorderPainter());
  final dynamic /*_InputBorderPainter*/ inputBorderPainter = customPaint.foregroundPainter;
  // ignore: avoid_dynamic_calls
  final dynamic /*_InputBorderTween*/ inputBorderTween = inputBorderPainter.border;
  // ignore: avoid_dynamic_calls
  final Animation<double> animation = inputBorderPainter.borderAnimation as Animation<double>;
  // ignore: avoid_dynamic_calls
  final InputBorder border = inputBorderTween.evaluate(animation) as InputBorder;
  return border;
}

BorderSide? getBorderSide(WidgetTester tester) {
  return getBorder(tester)!.borderSide;
}

BorderRadius? getBorderRadius(WidgetTester tester) {
  switch (getBorder(tester)!) {
    case UnderlineInputBorder(:final BorderRadius borderRadius):
    case OutlineInputBorder(:final BorderRadius borderRadius):
      return borderRadius;
  }
  return null;
}

double getBorderWeight(WidgetTester tester) => getBorderSide(tester)!.width;

Color getBorderColor(WidgetTester tester) => getBorderSide(tester)!.color;

Color getContainerColor(WidgetTester tester) {
  final CustomPaint customPaint = tester.widget(findBorderPainter());
  final dynamic /*_InputBorderPainter*/ inputBorderPainter = customPaint.foregroundPainter;
  // ignore: avoid_dynamic_calls
  return inputBorderPainter.blendedColor as Color;
}

double getOpacity(WidgetTester tester, String textValue) {
  final FadeTransition opacityWidget = tester.widget<FadeTransition>(
    find.ancestor(of: find.text(textValue), matching: find.byType(FadeTransition)).first,
  );
  return opacityWidget.opacity.value;
}

TextStyle? getIconStyle(WidgetTester tester, IconData icon) {
  final RichText iconRichText = tester.widget<RichText>(
    find.descendant(of: find.byIcon(icon), matching: find.byType(RichText)),
  );
  return iconRichText.text.style;
}

RenderObject getOverlayColor(WidgetTester tester) {
  return tester.allRenderObjects.firstWhere(
    (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
  );
}

void main() {
  // TODO(bleroux): migrate all M2 tests to M3.
  // See https://github.com/flutter/flutter/issues/139076
  // Work in progress.

  group('Material3 - InputDecoration container', () {
    // Default container height for InputDecorator (filled or outlined) is 56dp on mobile
    // whether the label is floating or not.
    // This value is taken from https://m3.material.io/components/text-fields/specs.
    const double containerHeight = 56.0;

    // On desktop, visual density is used to reduce the container height.
    // Desktop default density is [VisualDensity.compact] which corresponds to a density value of -2.
    // As a rule of thumb, a change of 1 or -1 in density corresponds to 4 logical pixels.
    // See https://m3.material.io/foundations/layout/understanding-layout/spacing#a5674a8b-5f38-4a58-8202-5838b082390d.
    const double desktopContainerHeight = containerHeight - 2 * 4.0; // 48.0

    group('for filled text field', () {
      group('when field is enabled', () {
        testWidgets('container has correct height and shape', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, containerHeight);
          expect(getBorder(tester), isA<UnderlineInputBorder>());
          expect(
            getBorderRadius(tester),
            const BorderRadius.only(topLeft: Radius.circular(4.0), topRight: Radius.circular(4.0)),
          );
        });

        testWidgets('container has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(
            findBorderPainter(),
            paints
              ..path(style: PaintingStyle.fill, color: theme.colorScheme.surfaceContainerHighest),
          );
        });

        testWidgets('active indicator has correct weight and color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getBorderColor(tester), theme.colorScheme.onSurfaceVariant);
          expect(getBorderWeight(tester), 1.0);
        });
      });

      group('when field is disabled', () {
        testWidgets('container has correct height and shape', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                enabled: false,
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, containerHeight);
          expect(getBorder(tester), isA<UnderlineInputBorder>());
          expect(
            getBorderRadius(tester),
            const BorderRadius.only(topLeft: Radius.circular(4.0), topRight: Radius.circular(4.0)),
          );
        });

        testWidgets('container has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                enabled: false,
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(
            findBorderPainter(),
            paints..path(
              style: PaintingStyle.fill,
              color: theme.colorScheme.onSurface.withOpacity(0.04),
            ),
          );
        });

        testWidgets('active indicator has correct weight and color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                enabled: false,
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getBorderColor(tester), theme.colorScheme.onSurface.withOpacity(0.38));
          expect(getBorderWeight(tester), 1.0);
        });
      });

      group('when field is hovered', () {
        testWidgets('container has correct height and shape', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, containerHeight);
          expect(getBorder(tester), isA<UnderlineInputBorder>());
          expect(
            getBorderRadius(tester),
            const BorderRadius.only(topLeft: Radius.circular(4.0), topRight: Radius.circular(4.0)),
          );
        });

        testWidgets('container has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(theme.hoverColor, Colors.black.withOpacity(0.04));
          expect(
            findBorderPainter(),
            paints..path(
              style: PaintingStyle.fill,
              color: Color.alphaBlend(theme.hoverColor, theme.colorScheme.surfaceContainerHighest),
            ),
          );
        });

        testWidgets('active indicator has correct weight and color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getBorderColor(tester), theme.colorScheme.onSurface);
          expect(getBorderWeight(tester), 1.0);
        });
      });

      group('when field is focused', () {
        testWidgets('container has correct height and shape', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, containerHeight);
          expect(getBorder(tester), isA<UnderlineInputBorder>());
          expect(
            getBorderRadius(tester),
            const BorderRadius.only(topLeft: Radius.circular(4.0), topRight: Radius.circular(4.0)),
          );
        });

        testWidgets('container has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(
            findBorderPainter(),
            paints
              ..path(style: PaintingStyle.fill, color: theme.colorScheme.surfaceContainerHighest),
          );
        });

        testWidgets('container has correct color when focused and hovered', (
          WidgetTester tester,
        ) async {
          // Regression test for https://github.com/flutter/flutter/issues/146573.
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              isHovering: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color focusColor = theme.colorScheme.surfaceContainerHighest;
          final Color hoverColor = theme.hoverColor;
          expect(
            findBorderPainter(),
            paints
              ..path(style: PaintingStyle.fill, color: Color.alphaBlend(hoverColor, focusColor)),
          );
        });

        testWidgets('active indicator has correct weight and color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getBorderColor(tester), theme.colorScheme.primary);
          expect(getBorderWeight(tester), 2.0);
        });

        testWidgets('active indicator has correct weight and color when focused and hovered', (
          WidgetTester tester,
        ) async {
          // Regression test for https://github.com/flutter/flutter/issues/145897.
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              isHovering: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getBorderColor(tester), theme.colorScheme.primary);
          expect(getBorderWeight(tester), 2.0);
        });
      });

      group('when field is in error', () {
        testWidgets('container has correct height and shape', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                errorText: errorText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, containerHeight);
          expect(getBorder(tester), isA<UnderlineInputBorder>());
          expect(
            getBorderRadius(tester),
            const BorderRadius.only(topLeft: Radius.circular(4.0), topRight: Radius.circular(4.0)),
          );
        });

        testWidgets('container has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                errorText: errorText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(
            findBorderPainter(),
            paints
              ..path(style: PaintingStyle.fill, color: theme.colorScheme.surfaceContainerHighest),
          );
        });

        testWidgets('active indicator has correct weight and color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                errorText: errorText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getBorderColor(tester), theme.colorScheme.error);
          expect(getBorderWeight(tester), 1.0);
        });

        testWidgets('active indicator has correct weight and color when focused', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                errorText: errorText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getBorderColor(tester), theme.colorScheme.error);
          expect(getBorderWeight(tester), 2.0);
        });

        testWidgets('active indicator has correct weight and color when hovered', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                errorText: errorText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getBorderColor(tester), theme.colorScheme.onErrorContainer);
          expect(getBorderWeight(tester), 1.0);
        });

        testWidgets('active indicator has correct weight and color when focused and hovered', (
          WidgetTester tester,
        ) async {
          // Regression test for https://github.com/flutter/flutter/issues/145897.
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              isHovering: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                errorText: errorText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getBorderColor(tester), theme.colorScheme.error);
          expect(getBorderWeight(tester), 2.0);
        });
      });

      testWidgets('default container height is 48dp on desktop', (WidgetTester tester) async {
        await tester.pumpWidget(
          buildInputDecorator(
            decoration: const InputDecoration(
              filled: true,
              labelText: labelText,
              helperText: helperText,
            ),
          ),
        );

        expect(getContainerRect(tester).height, desktopContainerHeight);
      }, variant: TargetPlatformVariant.desktop());
    });

    group('for outlined text field', () {
      group('when field is enabled', () {
        testWidgets('container has correct height and shape', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, containerHeight);
          expect(getBorder(tester), isA<OutlineInputBorder>());
          expect(getBorderRadius(tester), const BorderRadius.all(Radius.circular(4.0)));
        });

        testWidgets('container is painted correctly', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          // Default outlined text field's container is not filled.
          expect(findBorderPainter(), paints..path(style: PaintingStyle.stroke));
        });

        testWidgets('outline has correct weight and color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getBorderColor(tester), theme.colorScheme.outline);
          expect(getBorderWeight(tester), 1.0);
        });
      });

      group('when field is disabled', () {
        testWidgets('container has correct height and shape', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                enabled: false,
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, containerHeight);
          expect(getBorder(tester), isA<OutlineInputBorder>());
          expect(getBorderRadius(tester), const BorderRadius.all(Radius.circular(4.0)));
        });

        testWidgets('container is painted correctly', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                enabled: false,
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          // Default outlined text field's container is not filled.
          expect(findBorderPainter(), paints..path(style: PaintingStyle.stroke));
        });

        testWidgets('outline has correct weight and color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                enabled: false,
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getBorderColor(tester), theme.colorScheme.onSurface.withOpacity(0.12));
          expect(getBorderWeight(tester), 1.0);
        });
      });

      group('when field is hovered', () {
        testWidgets('container has correct height and shape', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, containerHeight);
          expect(getBorder(tester), isA<OutlineInputBorder>());
          expect(getBorderRadius(tester), const BorderRadius.all(Radius.circular(4.0)));
        });

        testWidgets('container is painted correctly', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          // Default outlined text field's container is not filled.
          expect(findBorderPainter(), paints..path(style: PaintingStyle.stroke));
        });

        testWidgets('outline has correct weight and color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getBorderColor(tester), theme.colorScheme.onSurface);
          expect(getBorderWeight(tester), 1.0);
        });
      });

      group('when field is focused', () {
        testWidgets('container has correct height and shape', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, containerHeight);
          expect(getBorder(tester), isA<OutlineInputBorder>());
          expect(getBorderRadius(tester), const BorderRadius.all(Radius.circular(4.0)));
        });

        testWidgets('container is painted correctly', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          // Default outlined text field's container is not filled.
          expect(findBorderPainter(), paints..path(style: PaintingStyle.stroke));
        });

        testWidgets('outline has correct weight and color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getBorderColor(tester), theme.colorScheme.primary);
          expect(getBorderWeight(tester), 2.0);
        });

        testWidgets('outline has correct weight and color when focused and hovered', (
          WidgetTester tester,
        ) async {
          // Regression test for https://github.com/flutter/flutter/issues/145897.
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              isHovering: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getBorderColor(tester), theme.colorScheme.primary);
          expect(getBorderWeight(tester), 2.0);
        });
      });

      group('when field is in error', () {
        testWidgets('container has correct height and shape', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                errorText: errorText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, containerHeight);
          expect(getBorder(tester), isA<OutlineInputBorder>());
          expect(getBorderRadius(tester), const BorderRadius.all(Radius.circular(4.0)));
        });

        testWidgets('container is painted correctly', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                errorText: errorText,
              ),
            ),
          );

          // Default outlined text field's container is not filled.
          expect(findBorderPainter(), paints..path(style: PaintingStyle.stroke));
        });

        testWidgets('outline has correct weight and color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                errorText: errorText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getBorderColor(tester), theme.colorScheme.error);
          expect(getBorderWeight(tester), 1.0);
        });

        testWidgets('outline has correct weight and color when focused', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                errorText: errorText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getBorderColor(tester), theme.colorScheme.error);
          expect(getBorderWeight(tester), 2.0);
        });

        testWidgets('outline has correct weight and color when hovered', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                errorText: errorText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getBorderColor(tester), theme.colorScheme.onErrorContainer);
          expect(getBorderWeight(tester), 1.0);
        });

        testWidgets('outline has correct weight and color when focused and hovered', (
          WidgetTester tester,
        ) async {
          // Regression test for https://github.com/flutter/flutter/issues/145897.
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              isHovering: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                errorText: errorText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getBorderColor(tester), theme.colorScheme.error);
          expect(getBorderWeight(tester), 2.0);
        });
      });

      testWidgets('default container height is 48dp on desktop', (WidgetTester tester) async {
        await tester.pumpWidget(
          buildInputDecorator(
            decoration: const InputDecoration(
              border: OutlineInputBorder(),
              labelText: labelText,
              helperText: helperText,
            ),
          ),
        );

        expect(getContainerRect(tester).height, desktopContainerHeight);
      }, variant: TargetPlatformVariant.desktop());
    });

    testWidgets('InputDecorator with no input border', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecorator(
          isEmpty: true,
          decoration: const InputDecoration(border: InputBorder.none),
        ),
      );
      expect(getBorderWeight(tester), 0.0);
    });

    testWidgets('OutlineInputBorder radius carries over when lerping', (WidgetTester tester) async {
      // This is a regression test for https://github.com/flutter/flutter/issues/23982
      const Key key = Key('textField');

      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Directionality(
              textDirection: TextDirection.ltr,
              child: TextField(
                key: key,
                decoration: InputDecoration(
                  fillColor: Colors.white,
                  filled: true,
                  border: UnderlineInputBorder(
                    borderSide: BorderSide(color: Colors.blue, width: 2.0),
                    borderRadius: BorderRadius.zero,
                  ),
                ),
              ),
            ),
          ),
        ),
      );

      // TextField has the given border.
      expect(getBorderRadius(tester), BorderRadius.zero);

      // Focusing does not change the border.
      await tester.tap(find.byKey(key));
      await tester.pump();
      expect(getBorderRadius(tester), BorderRadius.zero);
      await tester.pump(const Duration(milliseconds: 100));
      expect(getBorderRadius(tester), BorderRadius.zero);
      await tester.pump(kTransitionDuration);
      expect(getBorderRadius(tester), BorderRadius.zero);
    });

    testWidgets('OutlineInputBorder async lerp', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/28724

      final Completer<void> completer = Completer<void>();
      bool waitIsOver = false;

      await tester.pumpWidget(
        MaterialApp(
          home: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return GestureDetector(
                onTap: () async {
                  setState(() {
                    waitIsOver = true;
                  });
                  await completer.future;
                  setState(() {
                    waitIsOver = false;
                  });
                },
                child: InputDecorator(
                  decoration: InputDecoration(
                    labelText: 'Test',
                    enabledBorder:
                        !waitIsOver
                            ? null
                            : const OutlineInputBorder(borderSide: BorderSide(color: Colors.blue)),
                  ),
                ),
              );
            },
          ),
        ),
      );

      await tester.tap(find.byType(StatefulBuilder));
      await tester.pump(kTransitionDuration);

      completer.complete();
      await tester.pump(kTransitionDuration);
    });

    test('InputBorder equality', () {
      // OutlineInputBorder's equality is defined by the borderRadius, borderSide, & gapPadding.
      const OutlineInputBorder outlineInputBorder = OutlineInputBorder(
        borderRadius: BorderRadius.all(Radius.circular(9.0)),
        borderSide: BorderSide(color: Colors.blue),
        gapPadding: 32.0,
      );
      expect(
        outlineInputBorder,
        const OutlineInputBorder(
          borderSide: BorderSide(color: Colors.blue),
          borderRadius: BorderRadius.all(Radius.circular(9.0)),
          gapPadding: 32.0,
        ),
      );
      expect(outlineInputBorder, isNot(const OutlineInputBorder()));
      expect(
        outlineInputBorder,
        isNot(
          const OutlineInputBorder(
            borderSide: BorderSide(color: Colors.red),
            borderRadius: BorderRadius.all(Radius.circular(9.0)),
            gapPadding: 32.0,
          ),
        ),
      );
      expect(
        outlineInputBorder,
        isNot(
          const OutlineInputBorder(
            borderSide: BorderSide(color: Colors.blue),
            borderRadius: BorderRadius.all(Radius.circular(10.0)),
            gapPadding: 32.0,
          ),
        ),
      );
      expect(
        outlineInputBorder,
        isNot(
          const OutlineInputBorder(
            borderSide: BorderSide(color: Colors.blue),
            borderRadius: BorderRadius.all(Radius.circular(9.0)),
            gapPadding: 33.0,
          ),
        ),
      );

      // UnderlineInputBorder's equality is defined by the borderSide and borderRadius.
      const UnderlineInputBorder underlineInputBorder = UnderlineInputBorder(
        borderSide: BorderSide(color: Colors.blue),
        borderRadius: BorderRadius.only(
          topLeft: Radius.circular(5.0),
          topRight: Radius.circular(5.0),
        ),
      );
      expect(
        underlineInputBorder,
        const UnderlineInputBorder(
          borderSide: BorderSide(color: Colors.blue),
          borderRadius: BorderRadius.only(
            topLeft: Radius.circular(5.0),
            topRight: Radius.circular(5.0),
          ),
        ),
      );
      expect(underlineInputBorder, isNot(const UnderlineInputBorder()));
      expect(
        underlineInputBorder,
        isNot(
          const UnderlineInputBorder(
            borderSide: BorderSide(color: Colors.red),
            borderRadius: BorderRadius.only(
              topLeft: Radius.circular(5.0),
              topRight: Radius.circular(5.0),
            ),
          ),
        ),
      );
      expect(
        underlineInputBorder,
        isNot(
          const UnderlineInputBorder(
            borderSide: BorderSide(color: Colors.blue),
            borderRadius: BorderRadius.only(
              topLeft: Radius.circular(6.0),
              topRight: Radius.circular(6.0),
            ),
          ),
        ),
      );
    });

    test('InputBorder hashCodes', () {
      // OutlineInputBorder's hashCode is defined by the borderRadius, borderSide, & gapPadding.
      const OutlineInputBorder outlineInputBorder = OutlineInputBorder(
        borderRadius: BorderRadius.all(Radius.circular(9.0)),
        borderSide: BorderSide(color: Colors.blue),
        gapPadding: 32.0,
      );
      expect(
        outlineInputBorder.hashCode,
        const OutlineInputBorder(
          borderRadius: BorderRadius.all(Radius.circular(9.0)),
          borderSide: BorderSide(color: Colors.blue),
          gapPadding: 32.0,
        ).hashCode,
      );
      expect(outlineInputBorder.hashCode, isNot(const OutlineInputBorder().hashCode));
      expect(
        outlineInputBorder.hashCode,
        isNot(
          const OutlineInputBorder(
            borderRadius: BorderRadius.all(Radius.circular(9.0)),
            borderSide: BorderSide(color: Colors.red),
            gapPadding: 32.0,
          ).hashCode,
        ),
      );
      expect(
        outlineInputBorder.hashCode,
        isNot(
          const OutlineInputBorder(
            borderRadius: BorderRadius.all(Radius.circular(10.0)),
            borderSide: BorderSide(color: Colors.blue),
            gapPadding: 32.0,
          ).hashCode,
        ),
      );
      expect(
        outlineInputBorder.hashCode,
        isNot(
          const OutlineInputBorder(
            borderRadius: BorderRadius.all(Radius.circular(9.0)),
            borderSide: BorderSide(color: Colors.blue),
            gapPadding: 33.0,
          ).hashCode,
        ),
      );

      // UnderlineInputBorder's hashCode is defined by the borderSide and borderRadius.
      const UnderlineInputBorder underlineInputBorder = UnderlineInputBorder(
        borderSide: BorderSide(color: Colors.blue),
        borderRadius: BorderRadius.only(
          topLeft: Radius.circular(5.0),
          topRight: Radius.circular(5.0),
        ),
      );
      expect(
        underlineInputBorder.hashCode,
        const UnderlineInputBorder(
          borderSide: BorderSide(color: Colors.blue),
          borderRadius: BorderRadius.only(
            topLeft: Radius.circular(5.0),
            topRight: Radius.circular(5.0),
          ),
        ).hashCode,
      );
      expect(
        underlineInputBorder.hashCode,
        isNot(
          const UnderlineInputBorder(
            borderSide: BorderSide(color: Colors.red),
            borderRadius: BorderRadius.only(
              topLeft: Radius.circular(5.0),
              topRight: Radius.circular(5.0),
            ),
          ).hashCode,
        ),
      );
      expect(
        underlineInputBorder.hashCode,
        isNot(
          const UnderlineInputBorder(
            borderSide: BorderSide(color: Colors.blue),
            borderRadius: BorderRadius.only(
              topLeft: Radius.circular(6.0),
              topRight: Radius.circular(6.0),
            ),
          ).hashCode,
        ),
      );
    });

    testWidgets('OutlineInputBorder borders scale down to fit when large values are passed in', (
      WidgetTester tester,
    ) async {
      // This is a regression test for https://github.com/flutter/flutter/issues/34327
      const double largerBorderRadius = 200.0;
      const double smallerBorderRadius = 100.0;
      const double inputDecoratorHeight = 56.0;
      const double inputDecoratorWidth = 800.0;

      await tester.pumpWidget(
        buildInputDecorator(
          decoration: const InputDecoration(
            filled: true,
            fillColor: Color(0xFF00FF00),
            labelText: 'label text',
            border: OutlineInputBorder(
              borderRadius: BorderRadius.only(
                // Intentionally large values that are larger than the InputDecorator.
                topLeft: Radius.circular(smallerBorderRadius),
                bottomLeft: Radius.circular(smallerBorderRadius),
                topRight: Radius.circular(largerBorderRadius),
                bottomRight: Radius.circular(largerBorderRadius),
              ),
            ),
          ),
        ),
      );

      // Skia determines the scale based on the ratios of radii to the total
      // height or width allowed. In this case, it is the right side of the
      // border, which have two corners with largerBorderRadius that add up
      // to be 400.0.
      const double denominator = largerBorderRadius * 2.0;

      const double largerBorderRadiusScaled =
          largerBorderRadius / denominator * inputDecoratorHeight;
      const double smallerBorderRadiusScaled =
          smallerBorderRadius / denominator * inputDecoratorHeight;

      expect(
        findBorderPainter(),
        paints
          ..save()
          ..path(
            style: PaintingStyle.fill,
            color: const Color(0xFF00FF00),
            includes: const <Offset>[
              // The border should draw along the four edges of the
              // InputDecorator.

              // Top center.
              Offset(inputDecoratorWidth / 2.0, 0.0),
              // Bottom center.
              Offset(inputDecoratorWidth / 2.0, inputDecoratorHeight),
              // Left center.
              Offset(0.0, inputDecoratorHeight / 2.0),
              // Right center.
              Offset(inputDecoratorWidth, inputDecoratorHeight / 2.0),

              // The border path should contain points where each rounded corner
              // ends.

              // Bottom-right arc.
              Offset(inputDecoratorWidth, inputDecoratorHeight - largerBorderRadiusScaled),
              Offset(inputDecoratorWidth - largerBorderRadiusScaled, inputDecoratorHeight),
              // Top-right arc.
              Offset(inputDecoratorWidth, 0.0 + largerBorderRadiusScaled),
              Offset(inputDecoratorWidth - largerBorderRadiusScaled, 0.0),
              // Bottom-left arc.
              Offset(0.0, inputDecoratorHeight - smallerBorderRadiusScaled),
              Offset(0.0 + smallerBorderRadiusScaled, inputDecoratorHeight),
              // Top-left arc.
              Offset(0.0, 0.0 + smallerBorderRadiusScaled),
              Offset(0.0 + smallerBorderRadiusScaled, 0.0),
            ],
            excludes: const <Offset>[
              // The border should not contain the corner points, since the border
              // is rounded.

              // Top-left.
              Offset.zero,
              // Top-right.
              Offset(inputDecoratorWidth, 0.0),
              // Bottom-left.
              Offset(0.0, inputDecoratorHeight),
              // Bottom-right.
              Offset(inputDecoratorWidth, inputDecoratorHeight),

              // Corners with larger border ratio should not contain points outside
              // of the larger radius.

              // Bottom-right arc.
              Offset(inputDecoratorWidth, inputDecoratorHeight - smallerBorderRadiusScaled),
              Offset(inputDecoratorWidth - smallerBorderRadiusScaled, inputDecoratorWidth),
              // Top-left arc.
              Offset(inputDecoratorWidth, 0.0 + smallerBorderRadiusScaled),
              Offset(inputDecoratorWidth - smallerBorderRadiusScaled, 0.0),
            ],
          )
          ..restore(),
      );
    }, skip: isBrowser); // https://github.com/flutter/flutter/issues/55317

    testWidgets('rounded OutlineInputBorder with zero padding just wraps the label', (
      WidgetTester tester,
    ) async {
      // This is a regression test for https://github.com/flutter/flutter/issues/82321
      const double borderRadius = 30.0;
      const String labelText = 'label text';

      const double inputDecoratorHeight = 56.0;
      const double inputDecoratorWidth = 800.0;

      await tester.pumpWidget(
        buildInputDecorator(
          decoration: InputDecoration(
            filled: true,
            fillColor: const Color(0xFF00FF00),
            labelText: labelText,
            border: OutlineInputBorder(
              borderRadius: BorderRadius.circular(borderRadius),
              gapPadding: 0.0,
            ),
          ),
        ),
      );

      const double denominator = borderRadius * 2.0;
      const double borderRadiusScaled = borderRadius / denominator * inputDecoratorHeight;

      expect(find.text(labelText), findsOneWidget);
      final Rect labelRect = tester.getRect(find.text(labelText));

      expect(
        findBorderPainter(),
        paints
          ..save()
          ..path(
            style: PaintingStyle.fill,
            color: const Color(0xFF00FF00),
            includes: <Offset>[
              // The border should draw along the four edges of the
              // InputDecorator.

              // Top center.
              const Offset(inputDecoratorWidth / 2.0, 0.0),
              // Bottom center.
              const Offset(inputDecoratorWidth / 2.0, inputDecoratorHeight),
              // Left center.
              const Offset(0.0, inputDecoratorHeight / 2.0),
              // Right center.
              const Offset(inputDecoratorWidth, inputDecoratorHeight / 2.0),

              // The border path should contain points where each rounded corner
              // ends.

              // Bottom-right arc.
              const Offset(inputDecoratorWidth, inputDecoratorHeight - borderRadiusScaled),
              const Offset(inputDecoratorWidth - borderRadiusScaled, inputDecoratorHeight),
              // Top-right arc.
              const Offset(inputDecoratorWidth, 0.0 + borderRadiusScaled),
              const Offset(inputDecoratorWidth - borderRadiusScaled, 0.0),
              // Bottom-left arc.
              const Offset(0.0, inputDecoratorHeight - borderRadiusScaled),
              const Offset(0.0 + borderRadiusScaled, inputDecoratorHeight),
              // Top-left arc.
              const Offset(0.0, 0.0 + borderRadiusScaled),
              const Offset(0.0 + borderRadiusScaled, 0.0),

              // Gap edges:
              // gap start x = radius - radius * cos(arc sweep).
              // gap start y = radius - radius * sin(arc sweep).
              const Offset(39.49999999999999, 32.284366616798906),
              Offset(39.49999999999999 + labelRect.width, 0.0),
            ],
            excludes: const <Offset>[
              // The border should not contain the corner points, since the border
              // is rounded.

              // Top-left.
              Offset.zero,
              // Top-right.
              Offset(inputDecoratorWidth, 0.0),
              // Bottom-left.
              Offset(0.0, inputDecoratorHeight),
              // Bottom-right.
              Offset(inputDecoratorWidth, inputDecoratorHeight),
            ],
          )
          ..restore(),
      );
    }, skip: isBrowser); // https://github.com/flutter/flutter/issues/55317

    testWidgets('OutlineInputBorder with BorderRadius.zero should draw a rectangular border', (
      WidgetTester tester,
    ) async {
      // Regression test for https://github.com/flutter/flutter/issues/78855.
      const String labelText = 'Flutter';
      const double inputDecoratorHeight = 56.0;
      const double inputDecoratorWidth = 800.0;
      const double borderWidth = 4.0;

      await tester.pumpWidget(
        buildInputDecorator(
          isFocused: true,
          decoration: const InputDecoration(
            filled: false,
            labelText: labelText,
            focusedBorder: OutlineInputBorder(
              borderRadius: BorderRadius.zero,
              borderSide: BorderSide(width: borderWidth, color: Colors.red),
            ),
          ),
        ),
      );

      expect(find.text(labelText), findsOneWidget);
      expect(
        findBorderPainter(),
        paints
          ..save()
          ..path(
            includes: const <Offset>[
              // Corner points in the middle of the border line should be in the path.
              // The path is not filled and borderWidth is 4.0 so Offset(2.0, 2.0) is in the path and Offset(1.0, 1.0) is not.
              // See Skia SkPath::contains method.

              // Top-left.
              Offset(borderWidth / 2, borderWidth / 2),
              // Top-right.
              Offset(inputDecoratorWidth - 1 - borderWidth / 2, borderWidth / 2),
              // Bottom-left.
              Offset(borderWidth / 2, inputDecoratorHeight - 1 - borderWidth / 2),
              // Bottom-right.
              Offset(
                inputDecoratorWidth - 1 - borderWidth / 2,
                inputDecoratorHeight - 1 - borderWidth / 2,
              ),
            ],
            excludes: const <Offset>[
              // The path is not filled and borderWidth is 4.0 so the path should not contains the corner points.
              // See Skia SkPath::contains method.

              // Top-left.
              Offset.zero,
              // // Top-right.
              Offset(inputDecoratorWidth - 1, 0),
              // // Bottom-left.
              Offset(0, inputDecoratorHeight - 1),
              // // Bottom-right.
              Offset(inputDecoratorWidth - 1, inputDecoratorHeight - 1),
            ],
          )
          ..restore(),
      );
    }, skip: isBrowser); // https://github.com/flutter/flutter/issues/55317

    testWidgets('InputDecorator OutlineInputBorder fillColor is clipped by border', (
      WidgetTester tester,
    ) async {
      // This is a regression test for https://github.com/flutter/flutter/issues/15742
      await tester.pumpWidget(
        buildInputDecorator(
          decoration: const InputDecoration(
            filled: true,
            fillColor: Color(0xFF00FF00),
            border: OutlineInputBorder(borderRadius: BorderRadius.all(Radius.circular(12.0))),
          ),
        ),
      );

      final RenderBox box = tester.renderObject(find.byType(InputDecorator));

      // Fill is the border's outer path, a rounded rectangle.
      expect(
        box,
        paints..path(
          style: PaintingStyle.fill,
          color: const Color(0xFF00FF00),
          includes: <Offset>[const Offset(800.0 / 2.0, 56 / 2.0)],
          excludes: <Offset>[
            const Offset(1.0, 6.0), // outside the rounded corner, top left.
            const Offset(800.0 - 1.0, 6.0), // top right.
            const Offset(1.0, 56.0 - 6.0), // bottom left.
            const Offset(800 - 1.0, 56.0 - 6.0), // bottom right.
          ],
        ),
      );

      // Border outline. The rrect is the -center- of the 1.0 stroked outline.
      expect(
        box,
        paints..rrect(
          style: PaintingStyle.stroke,
          strokeWidth: 1.0,
          rrect: RRect.fromLTRBR(0.5, 0.5, 799.5, 55.5, const Radius.circular(11.5)),
        ),
      );
    });

    testWidgets('InputDecorator UnderlineInputBorder fillColor is clipped by border', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        buildInputDecorator(
          decoration: const InputDecoration(
            filled: true,
            fillColor: Color(0xFF00FF00),
            border: UnderlineInputBorder(
              borderRadius: BorderRadius.only(
                bottomLeft: Radius.circular(12.0),
                bottomRight: Radius.circular(12.0),
              ),
            ),
          ),
        ),
      );

      final RenderBox box = tester.renderObject(find.byType(InputDecorator));

      // Fill is the border's outer path, a rounded rectangle.
      expect(
        box,
        paints..drrect(
          style: PaintingStyle.fill,
          inner: RRect.fromLTRBAndCorners(
            0.0,
            0.0,
            800.0,
            47.0,
            bottomRight: const Radius.elliptical(12.0, 11.0),
            bottomLeft: const Radius.elliptical(12.0, 11.0),
          ),
          outer: RRect.fromLTRBAndCorners(
            0.0,
            0.0,
            800.0,
            48.0,
            bottomRight: const Radius.elliptical(12.0, 12.0),
            bottomLeft: const Radius.elliptical(12.0, 12.0),
          ),
        ),
      );
    });

    testWidgets('UnderlineInputBorder clips top border to prevent anti-aliasing glitches', (
      WidgetTester tester,
    ) async {
      const Rect canvasRect = Rect.fromLTWH(0, 0, 100, 100);
      const UnderlineInputBorder border = UnderlineInputBorder(
        borderRadius: BorderRadius.all(Radius.circular(12.0)),
      );
      expect(
        (Canvas canvas) => border.paint(canvas, canvasRect),
        paints..drrect(
          outer: RRect.fromLTRBAndCorners(
            0.0,
            0.0,
            100.0,
            100.0,
            bottomRight: const Radius.elliptical(12.0, 12.0),
            bottomLeft: const Radius.elliptical(12.0, 12.0),
          ),
          inner: RRect.fromLTRBAndCorners(
            0.0,
            0.0,
            100.0,
            99.0,
            bottomRight: const Radius.elliptical(12.0, 11.0),
            bottomLeft: const Radius.elliptical(12.0, 11.0),
          ),
        ),
      );

      const UnderlineInputBorder border2 = UnderlineInputBorder(
        borderRadius: BorderRadius.all(Radius.circular(60.0)),
      );
      expect(
        (Canvas canvas) => border2.paint(canvas, canvasRect),
        paints..drrect(
          outer: RRect.fromLTRBAndCorners(
            0.0,
            0.0,
            100.0,
            100.0,
            bottomRight: const Radius.elliptical(50.0, 50.0),
            bottomLeft: const Radius.elliptical(50.0, 50.0),
          ),
          inner: RRect.fromLTRBAndCorners(
            0.0,
            0.0,
            100.0,
            99.0,
            bottomRight: const Radius.elliptical(50.0, 49.0),
            bottomLeft: const Radius.elliptical(50.0, 49.0),
          ),
        ),
        reason: 'clamp is expected',
      );
    });

    testWidgets('UnderlineInputBorder draws bottom border inside container bounds', (
      WidgetTester tester,
    ) async {
      const Rect canvasRect = Rect.fromLTWH(0, 0, 100, 100);
      const double borderWidth = 2.0;
      const UnderlineInputBorder border = UnderlineInputBorder(
        borderSide: BorderSide(width: borderWidth),
      );
      expect(
        (Canvas canvas) => border.paint(canvas, canvasRect),
        paints..line(
          p1: Offset(0, canvasRect.height - borderWidth / 2),
          p2: Offset(100, canvasRect.height - borderWidth / 2),
          strokeWidth: borderWidth,
        ),
      );
    });

    // Regression test for https://github.com/flutter/flutter/issues/82321.
    // Regression test for https://github.com/flutter/flutter/issues/150109.
    testWidgets(
      'OutlineBorder starts at the right position when border radius is taller than horizontal content padding',
      (WidgetTester tester) async {
        Widget buildFrame(TextDirection textDirection) {
          return MaterialApp(
            home: Scaffold(
              body: Container(
                padding: const EdgeInsets.all(16.0),
                alignment: Alignment.center,
                child: Directionality(
                  textDirection: textDirection,
                  child: RepaintBoundary(
                    child: InputDecorator(
                      isFocused: true,
                      isEmpty: true,
                      decoration: InputDecoration(
                        labelText: labelText,
                        border: OutlineInputBorder(
                          borderRadius: BorderRadius.circular(30.0),
                          gapPadding: 0.0,
                        ),
                      ),
                    ),
                  ),
                ),
              ),
            ),
          );
        }

        await tester.pumpWidget(buildFrame(TextDirection.ltr));
        RenderBox borderBox = InputDecorator.containerOf(tester.element(findBorderPainter()))!;
        // Convert label bottom left offset to border path coordinate system.
        final Offset labelBottomLeftLocalToBorder = borderBox.globalToLocal(
          getLabelRect(tester).bottomLeft,
        );

        expect(
          findBorderPainter(),
          paints
            ..save()
            ..path(
              // The label bottom left corner should not be part of the border.
              excludes: <Offset>[labelBottomLeftLocalToBorder],
              // The points just before the label bottom left corner should be part of the border.
              includes: <Offset>[
                labelBottomLeftLocalToBorder + const Offset(-1, 0),
                labelBottomLeftLocalToBorder + const Offset(-1, -1),
              ],
            )
            ..restore(),
        );

        await tester.pumpWidget(buildFrame(TextDirection.rtl));
        borderBox = InputDecorator.containerOf(tester.element(findBorderPainter()))!;
        // Convert label bottom right offset to border path coordinate system.
        final Offset labelBottomRightLocalToBorder = borderBox.globalToLocal(
          getLabelRect(tester).bottomRight,
        );

        expect(
          findBorderPainter(),
          paints
            ..save()
            ..path(
              // The label bottom right corner should not be part of the border.
              excludes: <Offset>[labelBottomRightLocalToBorder],
              // The points just after the label bottom right corner should be part of the border.
              includes: <Offset>[
                labelBottomRightLocalToBorder + const Offset(1, 0),
                labelBottomRightLocalToBorder + const Offset(1, -1),
              ],
            )
            ..restore(),
        );
      },
    );

    // Regression test for https://github.com/flutter/flutter/issues/159942.
    testWidgets('OutlineBorder does not overlap with the label at the default radius', (
      WidgetTester tester,
    ) async {
      Widget buildFrame(TextDirection textDirection) {
        return MaterialApp(
          home: Scaffold(
            body: Container(
              padding: const EdgeInsets.all(16.0),
              alignment: Alignment.center,
              child: Directionality(
                textDirection: textDirection,
                child: const RepaintBoundary(
                  child: InputDecorator(
                    isFocused: true,
                    decoration: InputDecoration(
                      labelText: labelText,
                      border: OutlineInputBorder(gapPadding: 0.0),
                    ),
                  ),
                ),
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildFrame(TextDirection.ltr));
      Rect labelRect = getLabelRect(tester);
      RenderBox borderBox = InputDecorator.containerOf(tester.element(findBorderPainter()))!;
      expect(
        findBorderPainter(),
        paints
          ..save()
          ..path(
            // The points of the label edge should be part of the border.
            includes: <Offset>[
              borderBox.globalToLocal(labelRect.centerLeft),
              borderBox.globalToLocal(labelRect.centerRight),
            ],
            // The points inside the label should not be part of the border.
            excludes: <Offset>[
              borderBox.globalToLocal(labelRect.centerLeft) + const Offset(1, 0),
              borderBox.globalToLocal(labelRect.centerRight) + const Offset(-1, 0),
            ],
          )
          ..restore(),
      );

      await tester.pumpWidget(buildFrame(TextDirection.rtl));
      labelRect = getLabelRect(tester);
      borderBox = InputDecorator.containerOf(tester.element(findBorderPainter()))!;
      expect(
        findBorderPainter(),
        paints
          ..save()
          ..path(
            // The points of the label edge should be part of the border.
            includes: <Offset>[
              borderBox.globalToLocal(labelRect.centerLeft),
              borderBox.globalToLocal(labelRect.centerRight),
            ],
            // The points inside the label should not be part of the border.
            excludes: <Offset>[
              borderBox.globalToLocal(labelRect.centerLeft) + const Offset(1, 0),
              borderBox.globalToLocal(labelRect.centerRight) + const Offset(-1, 0),
            ],
          )
          ..restore(),
      );
    });

    testWidgets(
      'OutlineBorder does not draw over label when input decorator is focused and has an icon',
      (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/18111.
        Widget buildFrame(TextDirection textDirection) {
          return MaterialApp(
            home: Scaffold(
              body: Container(
                padding: const EdgeInsets.all(16.0),
                alignment: Alignment.center,
                child: Directionality(
                  textDirection: textDirection,
                  child: const RepaintBoundary(
                    child: InputDecorator(
                      isFocused: true,
                      isEmpty: true,
                      decoration: InputDecoration(
                        icon: Icon(Icons.insert_link),
                        labelText: labelText,
                        border: OutlineInputBorder(),
                      ),
                    ),
                  ),
                ),
              ),
            ),
          );
        }

        await tester.pumpWidget(buildFrame(TextDirection.ltr));
        RenderBox borderBox = InputDecorator.containerOf(tester.element(findBorderPainter()))!;
        expect(
          findBorderPainter(),
          paints
            ..save()
            ..path(
              excludes: <Offset>[
                borderBox.globalToLocal(getLabelRect(tester).centerLeft),
                borderBox.globalToLocal(getLabelRect(tester).centerRight),
              ],
            )
            ..restore(),
        );

        await tester.pumpWidget(buildFrame(TextDirection.rtl));
        borderBox = InputDecorator.containerOf(tester.element(findBorderPainter()))!;
        expect(
          findBorderPainter(),
          paints
            ..save()
            ..path(
              excludes: <Offset>[
                borderBox.globalToLocal(getLabelRect(tester).centerLeft),
                borderBox.globalToLocal(getLabelRect(tester).centerRight),
              ],
            )
            ..restore(),
        );
      },
    );
  });

  group('Material3 - InputDecoration label', () {
    group('for filled text field', () {
      group('when field is enabled', () {
        testWidgets('label text has correct style', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(filled: true, labelText: labelText),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          // Current input decorator implementation forces line height to 1.0,
          // this is not compliant with M3 spec.
          final TextStyle expectedStyle = theme.textTheme.bodyLarge!.copyWith(
            color: expectedColor,
            height: 1.0,
          );
          expect(getLabelStyle(tester), expectedStyle);
        });
      });

      group('when field is disabled', () {
        testWidgets('label text has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(filled: true, enabled: false, labelText: labelText),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getLabelStyle(tester).color, theme.colorScheme.onSurface.withOpacity(0.38));
        });
      });

      group('when field is hovered', () {
        testWidgets('label text has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(filled: true, labelText: labelText),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getLabelStyle(tester).color, theme.colorScheme.onSurfaceVariant);
        });
      });

      group('when field is focused', () {
        testWidgets('label text has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(filled: true, labelText: labelText),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getLabelStyle(tester).color, theme.colorScheme.primary);
        });

        testWidgets('label text has correct color when focused and hovered', (
          WidgetTester tester,
        ) async {
          // Regression test for https://github.com/flutter/flutter/issues/146565.
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              isHovering: true,
              decoration: const InputDecoration(filled: true, labelText: labelText),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getLabelStyle(tester).color, theme.colorScheme.primary);
        });
      });

      group('when field is in error', () {
        testWidgets('label text has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                errorText: errorText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getLabelStyle(tester).color, theme.colorScheme.error);
        });

        testWidgets('label text has correct color when focused', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                errorText: errorText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getLabelStyle(tester).color, theme.colorScheme.error);
        });

        testWidgets('label text has correct style when hovered', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                errorText: errorText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getLabelStyle(tester).color, theme.colorScheme.onErrorContainer);
        });

        testWidgets('label text has correct style when focused and hovered', (
          WidgetTester tester,
        ) async {
          // Regression test for https://github.com/flutter/flutter/issues/146565.
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              isHovering: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                errorText: errorText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getLabelStyle(tester).color, theme.colorScheme.error);
        });
      });
    });

    group('for outlined text field', () {
      group('when field is enabled', () {
        testWidgets('label text has correct style', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          // Current input decorator implementation forces line height to 1.0,
          // this is not compliant with M3 spec.
          final TextStyle expectedStyle = theme.textTheme.bodyLarge!.copyWith(
            color: expectedColor,
            height: 1.0,
          );
          expect(getLabelStyle(tester), expectedStyle);
        });
      });

      group('when field is disabled', () {
        testWidgets('label text has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                enabled: false,
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getLabelStyle(tester).color, theme.colorScheme.onSurface.withOpacity(0.38));
        });
      });

      group('when field is hovered', () {
        testWidgets('label text has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getLabelStyle(tester).color, theme.colorScheme.onSurfaceVariant);
        });
      });

      group('when field is focused', () {
        testWidgets('label text has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getLabelStyle(tester).color, theme.colorScheme.primary);
        });

        testWidgets('label text has correct color when focused and hovered', (
          WidgetTester tester,
        ) async {
          // Regression test for https://github.com/flutter/flutter/issues/146565.
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              isHovering: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getLabelStyle(tester).color, theme.colorScheme.primary);
        });
      });

      group('when field is in error', () {
        testWidgets('label text has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                errorText: errorText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getLabelStyle(tester).color, theme.colorScheme.error);
        });

        testWidgets('label text has correct color when focused', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                errorText: errorText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getLabelStyle(tester).color, theme.colorScheme.error);
        });

        testWidgets('label text has correct color when hovered', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                errorText: errorText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getLabelStyle(tester).color, theme.colorScheme.onErrorContainer);
        });

        testWidgets('label text has correct color when focused and hovered', (
          WidgetTester tester,
        ) async {
          // Regression test for https://github.com/flutter/flutter/issues/146565.
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              isHovering: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                errorText: errorText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          expect(getLabelStyle(tester).color, theme.colorScheme.error);
        });
      });
    });

    testWidgets('floatingLabelStyle overrides default style', (WidgetTester tester) async {
      const TextStyle floatingLabelStyle = TextStyle(color: Colors.indigo, fontSize: 16.0);

      await tester.pumpWidget(
        buildInputDecorator(
          isEmpty: true,
          isFocused: true, // Label appears floating above input field.
          decoration: const InputDecoration(
            labelText: labelText,
            floatingLabelStyle: floatingLabelStyle,
          ),
        ),
      );

      expect(getLabelStyle(tester).color, floatingLabelStyle.color);
      expect(getLabelStyle(tester).fontSize, floatingLabelStyle.fontSize);
    });

    testWidgets('floatingLabelStyle defaults to labelStyle', (WidgetTester tester) async {
      const TextStyle labelStyle = TextStyle(color: Colors.amber, fontSize: 16.0);

      await tester.pumpWidget(
        buildInputDecorator(
          isEmpty: true,
          isFocused: true, // Label appears floating above input field.
          decoration: const InputDecoration(labelText: labelText, labelStyle: labelStyle),
        ),
      );

      expect(getLabelStyle(tester).color, labelStyle.color);
      expect(getLabelStyle(tester).fontSize, labelStyle.fontSize);
    });

    testWidgets('floatingLabelStyle takes precedence over labelStyle', (WidgetTester tester) async {
      const TextStyle labelStyle = TextStyle(color: Colors.amber, fontSize: 16.0);
      const TextStyle floatingLabelStyle = TextStyle(color: Colors.indigo, fontSize: 16.0);

      await tester.pumpWidget(
        buildInputDecorator(
          isEmpty: true,
          isFocused: true, // Label appears floating above input field.
          decoration: const InputDecoration(
            labelText: labelText,
            labelStyle: labelStyle,
            floatingLabelStyle: floatingLabelStyle,
          ),
        ),
      );

      expect(getLabelStyle(tester).color, floatingLabelStyle.color);
      expect(getLabelStyle(tester).fontSize, floatingLabelStyle.fontSize);
    });

    testWidgets('InputDecorationTheme labelStyle overrides default style', (
      WidgetTester tester,
    ) async {
      const TextStyle labelStyle = TextStyle(color: Colors.amber, fontSize: 16.0);

      await tester.pumpWidget(
        buildInputDecorator(
          isEmpty: true, // Label appears inline, on top of the input field.
          inputDecorationTheme: const InputDecorationTheme(labelStyle: labelStyle),
          decoration: const InputDecoration(labelText: labelText),
        ),
      );

      expect(getLabelStyle(tester).color, labelStyle.color);
    });

    testWidgets('InputDecorationTheme floatingLabelStyle overrides default style', (
      WidgetTester tester,
    ) async {
      const TextStyle floatingLabelStyle = TextStyle(color: Colors.indigo, fontSize: 16.0);

      await tester.pumpWidget(
        buildInputDecorator(
          isEmpty: true,
          isFocused: true, // Label appears floating above input field.
          inputDecorationTheme: const InputDecorationTheme(floatingLabelStyle: floatingLabelStyle),
          decoration: const InputDecoration(labelText: labelText),
        ),
      );

      expect(getLabelStyle(tester).color, floatingLabelStyle.color);
    });

    testWidgets('floatingLabelStyle is always used when FloatingLabelBehavior.always', (
      WidgetTester tester,
    ) async {
      // Regression test for https://github.com/flutter/flutter/issues/147231.
      const TextStyle labelStyle = TextStyle(color: Colors.amber, fontSize: 16.0);
      const TextStyle floatingLabelStyle = TextStyle(color: Colors.indigo, fontSize: 16.0);

      await tester.pumpWidget(
        buildInputDecorator(
          isEmpty: true,
          decoration: const InputDecoration(
            labelText: labelText,
            labelStyle: labelStyle,
            floatingLabelStyle: floatingLabelStyle,
            floatingLabelBehavior: FloatingLabelBehavior.always,
          ),
        ),
      );

      expect(getLabelStyle(tester).color, floatingLabelStyle.color);
      expect(getLabelStyle(tester).fontSize, floatingLabelStyle.fontSize);

      // Focus the input decorator.
      await tester.pumpWidget(
        buildInputDecorator(
          isEmpty: true,
          isFocused: true,
          decoration: const InputDecoration(
            labelText: labelText,
            labelStyle: labelStyle,
            floatingLabelStyle: floatingLabelStyle,
            floatingLabelBehavior: FloatingLabelBehavior.always,
          ),
        ),
      );

      expect(getLabelStyle(tester).color, floatingLabelStyle.color);
      expect(getLabelStyle(tester).fontSize, floatingLabelStyle.fontSize);
    });
  });

  group('Material3 - InputDecoration labelText layout', () {
    testWidgets('The label appears above input', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecorator(decoration: const InputDecoration(labelText: labelText)),
      );

      // Overall height for this InputDecorator is 56dp on mobile:
      //    8 - top padding
      //   12 - floating label (font size = 16 * 0.75, line height is forced to 1.0)
      //    4 - gap between label and input (this is not part of the M3 spec)
      //   24 - input text (font size = 16, line height = 1.5)
      //    8 - bottom padding
      // TODO(bleroux): fix input decorator to not rely on a 4 pixels gap between the label and the input,
      // this gap is not compliant with the M3 spec (M3 spec uses line height for this purpose).
      expect(getDecoratorRect(tester).size, const Size(800.0, 56.0));
      expect(getLabelRect(tester).top, 8.0);
      expect(getLabelRect(tester).bottom, 20.0);
      expect(getInputRect(tester).top, 24.0);
      expect(getInputRect(tester).bottom, 48.0);
    });

    testWidgets('The label appears within the input when there is no text content', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        buildInputDecorator(isEmpty: true, decoration: const InputDecoration(labelText: labelText)),
      );

      expect(getDecoratorRect(tester).size, const Size(800.0, 56.0));
      // Label line height is forced to 1.0 and font size is 16.0,
      // the label should be vertically centered (20 pixels above and below).
      expect(getLabelRect(tester).top, 20.0);
      expect(getLabelRect(tester).bottom, 36.0);
      // From the M3 specification, centering the label is right, but setting the line height to 1.0 is not
      // compliant (the expected text style is bodyLarge which font size is 16.0 and its line height 1.5).
      // TODO(bleroux): fix input decorator to not rely on forcing the label text line height to 1.0.
    });

    testWidgets(
      'The label appears above the input when there is no content and floatingLabelBehavior is always',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          buildInputDecorator(
            isEmpty: true,
            decoration: const InputDecoration(
              labelText: labelText,
              floatingLabelBehavior: FloatingLabelBehavior.always,
            ),
          ),
        );

        expect(getDecoratorRect(tester).size, const Size(800.0, 56.0));
        expect(getLabelRect(tester).top, 8.0);
        expect(getLabelRect(tester).bottom, 20.0);
      },
    );

    testWidgets(
      'The label appears within the input text when there is content and floatingLabelBehavior is never',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          buildInputDecorator(
            decoration: const InputDecoration(
              labelText: labelText,
              floatingLabelBehavior: FloatingLabelBehavior.never,
            ),
          ),
        );

        expect(getDecoratorRect(tester).size, const Size(800.0, 56.0));
        expect(getLabelRect(tester).top, 20.0);
        expect(getLabelRect(tester).bottom, 36.0);
      },
    );

    testWidgets('Floating label animation duration and curve', (WidgetTester tester) async {
      Future<void> pumpInputDecorator({required bool isFocused}) async {
        return tester.pumpWidget(
          buildInputDecorator(
            isEmpty: true,
            isFocused: isFocused,
            decoration: const InputDecoration(
              labelText: labelText,
              floatingLabelBehavior: FloatingLabelBehavior.auto,
            ),
          ),
        );
      }

      await pumpInputDecorator(isFocused: false);
      expect(getLabelRect(tester).top, 20.0);

      // The label animates upwards and scales down.
      // The animation duration is 167ms and the curve is fastOutSlowIn.
      await pumpInputDecorator(isFocused: true);
      await tester.pump(const Duration(milliseconds: 42));
      expect(getLabelRect(tester).top, closeTo(17.09, 0.5));
      await tester.pump(const Duration(milliseconds: 42));
      expect(getLabelRect(tester).top, closeTo(10.66, 0.5));
      await tester.pump(const Duration(milliseconds: 42));
      expect(getLabelRect(tester).top, closeTo(8.47, 0.5));
      await tester.pump(const Duration(milliseconds: 41));
      expect(getLabelRect(tester).top, 8.0);

      // If the animation changes direction without first reaching the
      // AnimationStatus.completed or AnimationStatus.dismissed status,
      // the CurvedAnimation stays on the same curve in the opposite direction.
      // The pumpAndSettle is used to prevent this behavior.
      await tester.pumpAndSettle();

      // The label animates downwards and scales up.
      // The animation duration is 167ms and the curve is fastOutSlowIn.
      await pumpInputDecorator(isFocused: false);
      await tester.pump(const Duration(milliseconds: 42));
      expect(getLabelRect(tester).top, closeTo(10.90, 0.5));
      await tester.pump(const Duration(milliseconds: 42));
      expect(getLabelRect(tester).top, closeTo(17.34, 0.5));
      await tester.pump(const Duration(milliseconds: 42));
      expect(getLabelRect(tester).top, closeTo(19.69, 0.5));
      await tester.pump(const Duration(milliseconds: 41));
      expect(getLabelRect(tester).top, 20.0);
    });

    testWidgets('InputDecorator withdraws label when not empty or focused', (
      WidgetTester tester,
    ) async {
      Future<void> pumpDecorator({
        required bool focused,
        bool enabled = true,
        bool filled = false,
        bool empty = true,
        bool directional = false,
      }) async {
        return tester.pumpWidget(
          buildInputDecorator(
            isEmpty: empty,
            isFocused: focused,
            decoration: InputDecoration(
              labelText: 'Label',
              enabled: enabled,
              filled: filled,
              focusedBorder: const OutlineInputBorder(),
              disabledBorder: const OutlineInputBorder(),
              border: const OutlineInputBorder(),
            ),
          ),
        );
      }

      await pumpDecorator(focused: false);
      await tester.pump(kTransitionDuration);
      const Size labelSize = Size(82.5, 16);
      expect(getLabelRect(tester).topLeft, equals(const Offset(12, 20)));
      expect(getLabelRect(tester).size, equals(labelSize));

      await pumpDecorator(focused: false, empty: false);
      await tester.pump(kTransitionDuration);
      expect(getLabelRect(tester).topLeft, equals(const Offset(12, -5.5)));
      expect(getLabelRect(tester).size, equals(labelSize * 0.75));

      await pumpDecorator(focused: true);
      await tester.pump(kTransitionDuration);
      expect(getLabelRect(tester).topLeft, equals(const Offset(12, -5.5)));
      expect(getLabelRect(tester).size, equals(labelSize * 0.75));

      await pumpDecorator(focused: true, empty: false);
      await tester.pump(kTransitionDuration);
      expect(getLabelRect(tester).topLeft, equals(const Offset(12, -5.5)));
      expect(getLabelRect(tester).size, equals(labelSize * 0.75));

      await pumpDecorator(focused: false, enabled: false);
      await tester.pump(kTransitionDuration);
      expect(getLabelRect(tester).topLeft, equals(const Offset(12, 20)));
      expect(getLabelRect(tester).size, equals(labelSize));

      await pumpDecorator(focused: false, empty: false, enabled: false);
      await tester.pump(kTransitionDuration);
      expect(getLabelRect(tester).topLeft, equals(const Offset(12, -5.5)));
      expect(getLabelRect(tester).size, equals(labelSize * 0.75));

      // Focused and disabled happens with NavigationMode.directional.
      await pumpDecorator(focused: true, enabled: false);
      await tester.pump(kTransitionDuration);
      expect(getLabelRect(tester).topLeft, equals(const Offset(12, 20)));
      expect(getLabelRect(tester).size, equals(labelSize));

      await pumpDecorator(focused: true, empty: false, enabled: false);
      await tester.pump(kTransitionDuration);
      expect(getLabelRect(tester).topLeft, equals(const Offset(12, -5.5)));
      expect(getLabelRect(tester).size, equals(labelSize * 0.75));
    });

    testWidgets('InputDecorator floating label width scales when focused', (
      WidgetTester tester,
    ) async {
      final String longStringA = String.fromCharCodes(List<int>.generate(200, (_) => 65));
      final String longStringB = String.fromCharCodes(List<int>.generate(200, (_) => 66));

      await tester.pumpWidget(
        Center(
          child: SizedBox(
            width: 100,
            height: 100,
            child: buildInputDecorator(
              isEmpty: true,
              decoration: InputDecoration(labelText: longStringA),
            ),
          ),
        ),
      );

      expect(
        find.text(longStringA),
        paints..clipRect(rect: const Rect.fromLTWH(0, 0, 100.0, 16.0)),
      );

      await tester.pumpWidget(
        Center(
          child: SizedBox(
            width: 100,
            height: 100,
            child: buildInputDecorator(
              isFocused: true,
              isEmpty: true,
              decoration: InputDecoration(labelText: longStringB),
            ),
          ),
        ),
      );

      await tester.pump(kTransitionDuration);

      expect(
        find.text(longStringB),
        paints..something((Symbol methodName, List<dynamic> arguments) {
          if (methodName != #clipRect) {
            return false;
          }
          final Rect clipRect = arguments[0] as Rect;
          // _kFinalLabelScale = 0.75
          expect(
            clipRect,
            rectMoreOrLessEquals(const Rect.fromLTWH(0, 0, 100 / 0.75, 16.0), epsilon: 1e-5),
          );
          return true;
        }),
      );
    }, skip: isBrowser); // TODO(yjbanov): https://github.com/flutter/flutter/issues/44020

    testWidgets('InputDecorator floating label Y coordinate', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/54028
      await tester.pumpWidget(
        buildInputDecorator(
          isEmpty: true,
          decoration: const InputDecoration(
            labelText: labelText,
            enabledBorder: OutlineInputBorder(borderSide: BorderSide(width: 4)),
            floatingLabelBehavior: FloatingLabelBehavior.always,
          ),
        ),
      );

      await tester.pump(kTransitionDuration);

      // floatingLabelHeight = 12 (font size 16dps * 0.75 = 12)
      // labelY = -floatingLabelHeight/2 + borderWidth/2
      expect(getLabelRect(tester).top, -4.0);
    });

    testWidgets('InputDecorator respects reduced theme visualDensity', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecorator(
          isEmpty: true,
          visualDensity: VisualDensity.compact,
          decoration: const InputDecoration(labelText: labelText, hintText: hintText),
        ),
      );

      // Overall height for this InputDecorator is 48dp:
      //    4 - top padding (8 minus 4 due to reduced visual density)
      //   12 - floating label (font size = 16 * 0.75, line height is forced to 1.0)
      //    4 - gap between label and input (this is not part of the M3 spec)
      //   24 - input text (font size = 16, line height = 1.5)
      //    4 - bottom padding (8 minus 4 due to reduced visual density)
      expect(getDecoratorRect(tester).size, const Size(800.0, 48.0));

      // The decorator is empty, label is not floating and is vertically centered.
      expect(getLabelRect(tester).top, 16.0);
      expect(getLabelRect(tester).bottom, 32.0);
      expect(getHintOpacity(tester), 0.0);
      expect(getBorderBottom(tester), 48.0);
      expect(getBorderWeight(tester), 1.0);

      // When the decorator is focused, label moves upwards, hint is visible (opacity 1.0).
      await tester.pumpWidget(
        buildInputDecorator(
          isEmpty: true,
          isFocused: true,
          visualDensity: VisualDensity.compact,
          decoration: const InputDecoration(labelText: labelText, hintText: hintText),
        ),
      );
      await tester.pump(kTransitionDuration);

      // The decorator is empty and focused, label and hint are visible.
      expect(getDecoratorRect(tester).size, const Size(800.0, 48.0));
      expect(getLabelRect(tester).top, 4.0);
      expect(getLabelRect(tester).bottom, 16.0);
      expect(getHintRect(tester).top, 20.0);
      expect(getHintRect(tester).bottom, 44.0);
      expect(getHintOpacity(tester), 1.0);
      expect(getBorderBottom(tester), 48.0);
      expect(getBorderWeight(tester), 2.0);

      await tester.pumpWidget(
        buildInputDecorator(
          isFocused: true,
          visualDensity: VisualDensity.compact,
          decoration: const InputDecoration(labelText: labelText, hintText: hintText),
        ),
      );
      await tester.pump(kTransitionDuration);

      // The decorator is focused and not empty, label and input are visible.
      expect(getDecoratorRect(tester).size, const Size(800.0, 48.0));
      expect(getLabelRect(tester).top, 4.0);
      expect(getLabelRect(tester).bottom, 16.0);
      expect(getInputRect(tester).top, 20.0);
      expect(getInputRect(tester).bottom, 44.0);
      expect(getHintOpacity(tester), 0.0);
      expect(getBorderBottom(tester), 48.0);
      expect(getBorderWeight(tester), 2.0);
    });

    testWidgets('InputDecorator respects increased theme visualDensity', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        buildInputDecorator(
          isEmpty: true,
          visualDensity: const VisualDensity(horizontal: 2.0, vertical: 2.0),
          decoration: const InputDecoration(labelText: labelText, hintText: hintText),
        ),
      );

      // Overall height for this InputDecorator is 64dp:
      //   12 - top padding (8 plus 4 due to increased visual density)
      //   12 - floating label (font size = 16 * 0.75, line height is forced to 1.0)
      //    4 - gap between label and input (this is not part of the M3 spec)
      //   24 - input text (font size = 16, line height = 1.5)
      //   12 - bottom padding (8 plus 4 due to increased visual density)
      expect(getDecoratorRect(tester).size, const Size(800.0, 64.0));

      // The decorator is empty, label is not floating and is vertically centered.
      expect(getLabelRect(tester).top, 24.0);
      expect(getLabelRect(tester).bottom, 40.0);
      expect(getHintOpacity(tester), 0.0);
      expect(getBorderBottom(tester), 64.0);
      expect(getBorderWeight(tester), 1.0);

      // When the decorator is focused, label moves upwards, hint is visible (opacity 1.0).
      await tester.pumpWidget(
        buildInputDecorator(
          isEmpty: true,
          isFocused: true,
          visualDensity: const VisualDensity(horizontal: 2.0, vertical: 2.0),
          decoration: const InputDecoration(labelText: labelText, hintText: hintText),
        ),
      );
      await tester.pump(kTransitionDuration);

      // The decorator is empty and focused, label and hint are visible.
      expect(getDecoratorRect(tester).size, const Size(800.0, 64.0));
      expect(getLabelRect(tester).top, 12.0);
      expect(getLabelRect(tester).bottom, 24.0);
      expect(getHintRect(tester).top, 28.0);
      expect(getHintRect(tester).bottom, 52.0);
      expect(getHintOpacity(tester), 1.0);
      expect(getBorderBottom(tester), 64.0);
      expect(getBorderWeight(tester), 2.0);

      await tester.pumpWidget(
        buildInputDecorator(
          isFocused: true,
          visualDensity: const VisualDensity(horizontal: 2.0, vertical: 2.0),
          decoration: const InputDecoration(labelText: labelText, hintText: hintText),
        ),
      );
      await tester.pump(kTransitionDuration);

      // The decorator is focused and not empty, label and input are visible.
      expect(getDecoratorRect(tester).size, const Size(800.0, 64.0));
      expect(getLabelRect(tester).top, 12.0);
      expect(getLabelRect(tester).bottom, 24.0);
      expect(getInputRect(tester).top, 28.0);
      expect(getInputRect(tester).bottom, 52.0);
      expect(getHintOpacity(tester), 0.0);
      expect(getBorderBottom(tester), 64.0);
      expect(getBorderWeight(tester), 2.0);
    });
  });

  group('Material3 - InputDecoration label layout', () {
    testWidgets('The label appears above input', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecorator(decoration: const InputDecoration(label: customLabel)),
      );

      // Overall height for this InputDecorator is 56dp on mobile:
      //    8 - top padding
      //   12 - floating label (font size = 16 * 0.75, line height is forced to 1.0)
      //    4 - gap between label and input (this is not part of the M3 spec)
      //   24 - input text (font size = 16, line height = 1.5)
      //    8 - bottom padding
      expect(getDecoratorRect(tester).size, const Size(800.0, 56.0));
      expect(getCustomLabelRect(tester).top, 8.0);
      expect(getCustomLabelRect(tester).bottom, 20.0);
      expect(getInputRect(tester).top, 24.0);
      expect(getInputRect(tester).bottom, 48.0);
    });

    testWidgets('The label appears within the input when there is no text content', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        buildInputDecorator(isEmpty: true, decoration: const InputDecoration(label: customLabel)),
      );

      expect(getDecoratorRect(tester).size, const Size(800.0, 56.0));
      // Label line height is forced to 1.0 and font size is 16.0,
      // the label should be vertically centered (20 pixels above and below).
      expect(getCustomLabelRect(tester).top, 20.0);
      expect(getCustomLabelRect(tester).bottom, 36.0);
    });

    testWidgets(
      'The label appears above the input when there is no content and floatingLabelBehavior is always',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          buildInputDecorator(
            isEmpty: true,
            decoration: const InputDecoration(
              label: customLabel,
              floatingLabelBehavior: FloatingLabelBehavior.always,
            ),
          ),
        );

        expect(getDecoratorRect(tester).size, const Size(800.0, 56.0));
        //  8 - top padding
        // 12 - floating label height (font size = 16 * 0.75, line height is forced to 1.0)
        expect(getCustomLabelRect(tester).top, 8.0);
        expect(getCustomLabelRect(tester).bottom, 20.0);
      },
    );

    testWidgets(
      'The label appears within the input text when there is content and floatingLabelBehavior is never',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          buildInputDecorator(
            decoration: const InputDecoration(
              label: customLabel,
              floatingLabelBehavior: FloatingLabelBehavior.never,
            ),
          ),
        );

        expect(getDecoratorRect(tester).size, const Size(800.0, 56.0));
        // Label line height is forced to 1.0 and font size is 16.0,
        // the label should be vertically centered (20 pixels above and below).
        expect(getCustomLabelRect(tester).top, 20.0);
        expect(getCustomLabelRect(tester).bottom, 36.0);
      },
    );

    testWidgets('Floating label animation duration and curve', (WidgetTester tester) async {
      Future<void> pumpInputDecorator({required bool isFocused}) async {
        return tester.pumpWidget(
          buildInputDecorator(
            isEmpty: true,
            isFocused: isFocused,
            decoration: const InputDecoration(
              label: customLabel,
              floatingLabelBehavior: FloatingLabelBehavior.auto,
            ),
          ),
        );
      }

      await pumpInputDecorator(isFocused: false);
      // Label line height is forced to 1.0 and font size is 16.0,
      // the label should be vertically centered (20 pixels above and below).
      expect(getCustomLabelRect(tester).top, 20.0);

      // The label animates upwards and scales down.
      // The animation duration is 167ms and the curve is fastOutSlowIn.
      await pumpInputDecorator(isFocused: true);
      await tester.pump(const Duration(milliseconds: 42));
      expect(getCustomLabelRect(tester).top, closeTo(17.09, 0.5));
      await tester.pump(const Duration(milliseconds: 42));
      expect(getCustomLabelRect(tester).top, closeTo(10.66, 0.5));
      await tester.pump(const Duration(milliseconds: 42));
      expect(getCustomLabelRect(tester).top, closeTo(8.47, 0.5));
      await tester.pump(const Duration(milliseconds: 41));
      expect(getCustomLabelRect(tester).top, 8.0);

      // If the animation changes direction without first reaching the
      // AnimationStatus.completed or AnimationStatus.dismissed status,
      // the CurvedAnimation stays on the same curve in the opposite direction.
      // The pumpAndSettle is used to prevent this behavior.
      await tester.pumpAndSettle();

      // The label animates downwards and scales up.
      // The animation duration is 167ms and the curve is fastOutSlowIn.
      await pumpInputDecorator(isFocused: false);
      await tester.pump(const Duration(milliseconds: 42));
      expect(getCustomLabelRect(tester).top, closeTo(10.90, 0.5));
      await tester.pump(const Duration(milliseconds: 42));
      expect(getCustomLabelRect(tester).top, closeTo(17.34, 0.5));
      await tester.pump(const Duration(milliseconds: 42));
      expect(getCustomLabelRect(tester).top, closeTo(19.69, 0.5));
      await tester.pump(const Duration(milliseconds: 41));
      expect(getCustomLabelRect(tester).top, 20.0);
    });

    testWidgets(
      'InputDecorationTheme floatingLabelStyle overrides label widget styles when the widget is a text widget (focused)',
      (WidgetTester tester) async {
        const TextStyle style16 = TextStyle(fontSize: 16.0);
        final TextStyle floatingLabelStyle = style16.merge(const TextStyle(color: Colors.indigo));

        await tester.pumpWidget(
          buildInputDecorator(
            isEmpty: true,
            isFocused: true, // Label appears floating above input field.
            inputDecorationTheme: InputDecorationTheme(floatingLabelStyle: floatingLabelStyle),
            decoration: const InputDecoration(label: Text.rich(TextSpan(text: 'label'))),
          ),
        );

        // Verify that the styles were passed along.
        expect(getLabelStyle(tester).color, floatingLabelStyle.color);
      },
    );

    testWidgets(
      'InputDecorationTheme labelStyle overrides label widget styles when the widget is a text widget',
      (WidgetTester tester) async {
        const TextStyle styleDefaultSize = TextStyle(fontSize: 16.0);
        final TextStyle labelStyle = styleDefaultSize.merge(const TextStyle(color: Colors.purple));

        await tester.pumpWidget(
          buildInputDecorator(
            isEmpty: true, // Label appears inline, on top of the input field.
            inputDecorationTheme: InputDecorationTheme(labelStyle: labelStyle),
            decoration: const InputDecoration(label: Text.rich(TextSpan(text: 'label'))),
          ),
        );

        // Verify that the styles were passed along.
        expect(getLabelStyle(tester).color, labelStyle.color);
      },
    );
  });

  group('Material3 - InputDecoration hint', () {
    group('for filled text field without label', () {
      // Overall height for this InputDecorator is 48dp on mobile:
      //   12 - Top padding
      //   24 - Input and hint (font size = 16, line height = 1.5)
      //   12 - Bottom padding
      group('when field is enabled', () {
        testWidgets('hint and input align vertically when decorator is empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isEmpty: true,
              decoration: const InputDecoration(filled: true, hintText: hintText),
            ),
          );

          expect(getContainerRect(tester).height, 48.0);
          expect(getInputRect(tester).top, 12.0);
          expect(getInputRect(tester).bottom, 36.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is visible because decorator is empty and there is no label.
          expect(getHintOpacity(tester), 1.0);
        });

        testWidgets('hint and input align vertically when decorator is not empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(filled: true, hintText: hintText),
            ),
          );

          expect(getContainerRect(tester).height, 48.0);
          expect(getInputRect(tester).top, 12.0);
          expect(getInputRect(tester).bottom, 36.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not empty.
          expect(getHintOpacity(tester), 0.0);
        });

        testWidgets('hint has correct style when visible', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isEmpty: true,
              decoration: const InputDecoration(filled: true, hintText: hintText),
            ),
          );

          // Hint is visible because decorator is empty.
          expect(getHintOpacity(tester), 1.0);

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          final TextStyle expectedStyle = theme.textTheme.bodyLarge!.copyWith(color: expectedColor);
          expect(getHintStyle(tester), expectedStyle);
        });
      });

      group('when field is disabled', () {
        testWidgets('hint and input align vertically when decorator is empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isEmpty: true,
              decoration: const InputDecoration(enabled: false, filled: true, hintText: hintText),
            ),
          );

          expect(getContainerRect(tester).height, 48.0);
          expect(getInputRect(tester).top, 12.0);
          expect(getInputRect(tester).bottom, 36.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is visible because decorator is empty and there is no label.
          expect(getHintOpacity(tester), 1.0);
        });

        testWidgets('hint and input align vertically when decorator is not empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(enabled: false, filled: true, hintText: hintText),
            ),
          );

          expect(getContainerRect(tester).height, 48.0);
          expect(getInputRect(tester).top, 12.0);
          expect(getInputRect(tester).bottom, 36.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not empty.
          expect(getHintOpacity(tester), 0.0);
        });

        testWidgets('hint has correct style when visible', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isEmpty: true,
              decoration: const InputDecoration(enabled: false, filled: true, hintText: hintText),
            ),
          );

          // Hint is visible because decorator is empty.
          expect(getHintOpacity(tester), 1.0);

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurface.withOpacity(0.38);
          final TextStyle expectedStyle = theme.textTheme.bodyLarge!.copyWith(color: expectedColor);
          expect(getHintStyle(tester), expectedStyle);
        });
      });

      group('when field is hovered', () {
        testWidgets('hint and input align vertically when decorator is empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              isEmpty: true,
              decoration: const InputDecoration(filled: true, hintText: hintText),
            ),
          );

          expect(getContainerRect(tester).height, 48.0);
          expect(getInputRect(tester).top, 12.0);
          expect(getInputRect(tester).bottom, 36.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is visible because decorator is empty and there is no label.
          expect(getHintOpacity(tester), 1.0);
        });

        testWidgets('hint and input align vertically when decorator is not empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(filled: true, hintText: hintText),
            ),
          );

          expect(getContainerRect(tester).height, 48.0);
          expect(getInputRect(tester).top, 12.0);
          expect(getInputRect(tester).bottom, 36.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not empty.
          expect(getHintOpacity(tester), 0.0);
        });

        testWidgets('hint has correct style when visible', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              isEmpty: true,
              decoration: const InputDecoration(filled: true, hintText: hintText),
            ),
          );

          // Hint is visible because decorator is empty.
          expect(getHintOpacity(tester), 1.0);

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          final TextStyle expectedStyle = theme.textTheme.bodyLarge!.copyWith(color: expectedColor);
          expect(getHintStyle(tester), expectedStyle);
        });
      });

      group('when field is focused', () {
        testWidgets('hint and input align vertically when decorator is empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              isEmpty: true,
              decoration: const InputDecoration(filled: true, hintText: hintText),
            ),
          );

          expect(getContainerRect(tester).height, 48.0);
          expect(getInputRect(tester).top, 12.0);
          expect(getInputRect(tester).bottom, 36.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is visible because decorator is empty and focused.
          expect(getHintOpacity(tester), 1.0);
        });

        testWidgets('hint and input align vertically when decorator is not empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(filled: true, hintText: hintText),
            ),
          );

          expect(getContainerRect(tester).height, 48.0);
          expect(getInputRect(tester).top, 12.0);
          expect(getInputRect(tester).bottom, 36.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not empty.
          expect(getHintOpacity(tester), 0.0);
        });

        testWidgets('hint has correct style when visible', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              isEmpty: true,
              decoration: const InputDecoration(filled: true, hintText: hintText),
            ),
          );

          // Hint is visible because decorator is empty.
          expect(getHintOpacity(tester), 1.0);

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          final TextStyle expectedStyle = theme.textTheme.bodyLarge!.copyWith(color: expectedColor);
          expect(getHintStyle(tester), expectedStyle);
        });
      });

      group('when field is in error', () {
        testWidgets('hint and input align vertically when decorator is empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isEmpty: true,
              decoration: const InputDecoration(
                filled: true,
                hintText: hintText,
                errorText: errorText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 48.0);
          expect(getInputRect(tester).top, 12.0);
          expect(getInputRect(tester).bottom, 36.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is visible because decorator is empty and there is no label.
          expect(getHintOpacity(tester), 1.0);
        });

        testWidgets('hint and input align vertically when decorator is not empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                hintText: hintText,
                errorText: errorText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 48.0);
          expect(getInputRect(tester).top, 12.0);
          expect(getInputRect(tester).bottom, 36.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not empty.
          expect(getHintOpacity(tester), 0.0);
        });

        testWidgets('hint has correct style when visible', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isEmpty: true,
              decoration: const InputDecoration(
                filled: true,
                hintText: hintText,
                errorText: errorText,
              ),
            ),
          );

          // Hint is visible because decorator is empty.
          expect(getHintOpacity(tester), 1.0);

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          final TextStyle expectedStyle = theme.textTheme.bodyLarge!.copyWith(color: expectedColor);
          expect(getHintStyle(tester), expectedStyle);
        });
      });
    });

    group('for filled text field with label', () {
      // Overall height for this InputDecorator is 56dp on mobile:
      //    8 - Top padding
      //   12 - Floating label (font size = 16 * 0.75, line height is forced to 1.0)
      //    4 - Gap between label and input (this is not part of the M3 spec)
      //   24 - Input/Hint (font size = 16, line height = 1.5)
      //    8 - Bottom padding
      group('when field is enabled', () {
        testWidgets('hint and input align vertically when decorator is empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isEmpty: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                hintText: hintText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 24.0);
          expect(getInputRect(tester).bottom, 48.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not focused (label is visible).
          expect(getHintOpacity(tester), 0.0);
        });

        testWidgets('hint and input align vertically when decorator is not empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                hintText: hintText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 24.0);
          expect(getInputRect(tester).bottom, 48.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not empty.
          expect(getHintOpacity(tester), 0.0);
        });
      });

      group('when field is disabled', () {
        testWidgets('hint and input align vertically when decorator is empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isEmpty: true,
              decoration: const InputDecoration(
                enabled: false,
                filled: true,
                labelText: labelText,
                hintText: hintText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 24.0);
          expect(getInputRect(tester).bottom, 48.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not focused (label is visible).
          expect(getHintOpacity(tester), 0.0);
        });

        testWidgets('hint and input align vertically when decorator is not empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                enabled: false,
                filled: true,
                labelText: labelText,
                hintText: hintText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 24.0);
          expect(getInputRect(tester).bottom, 48.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not empty.
          expect(getHintOpacity(tester), 0.0);
        });
      });

      group('when field is hovered', () {
        testWidgets('hint and input align vertically when decorator is empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              isEmpty: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                hintText: hintText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 24.0);
          expect(getInputRect(tester).bottom, 48.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not focused (label is visible).
          expect(getHintOpacity(tester), 0.0);
        });

        testWidgets('hint and input align vertically when decorator is not empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                hintText: hintText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 24.0);
          expect(getInputRect(tester).bottom, 48.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not empty.
          expect(getHintOpacity(tester), 0.0);
        });
      });

      group('when field is focused', () {
        testWidgets('hint and input align vertically when decorator is empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              isEmpty: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                hintText: hintText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 24.0);
          expect(getInputRect(tester).bottom, 48.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is visible because decorator is empty and focused.
          expect(getHintOpacity(tester), 1.0);
        });

        testWidgets('hint and input align vertically when decorator is not empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                hintText: hintText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 24.0);
          expect(getInputRect(tester).bottom, 48.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not empty.
          expect(getHintOpacity(tester), 0.0);
        });

        testWidgets('hint has correct style when visible', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              isEmpty: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                hintText: hintText,
              ),
            ),
          );

          // Hint is visible because decorator is empty.
          expect(getHintOpacity(tester), 1.0);

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          final TextStyle expectedStyle = theme.textTheme.bodyLarge!.copyWith(color: expectedColor);
          expect(getHintStyle(tester), expectedStyle);
        });
      });

      group('when field is in error', () {
        testWidgets('hint and input align vertically when decorator is empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isEmpty: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                hintText: hintText,
                errorText: errorText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 24.0);
          expect(getInputRect(tester).bottom, 48.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not focused (label is visible).
          expect(getHintOpacity(tester), 0.0);
        });

        testWidgets('hint and input align vertically when decorator is not empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                hintText: hintText,
                errorText: errorText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 24.0);
          expect(getInputRect(tester).bottom, 48.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not empty.
          expect(getHintOpacity(tester), 0.0);
        });
      });
    });

    group('for outlined text field without label', () {
      // Overall height for this InputDecorator is 56dp on mobile:
      //   16 - Top padding
      //   24 - Input and hint (font size = 16, line height = 1.5)
      //   16 - Bottom padding
      group('when field is enabled', () {
        testWidgets('hint and input align vertically when decorator is empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isEmpty: true,
              decoration: const InputDecoration(border: OutlineInputBorder(), hintText: hintText),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 16.0);
          expect(getInputRect(tester).bottom, 40.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is visible because decorator is empty.
          expect(getHintOpacity(tester), 1.0);
        });

        testWidgets('hint and input align vertically when decorator is not empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(border: OutlineInputBorder(), hintText: hintText),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 16.0);
          expect(getInputRect(tester).bottom, 40.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not empty.
          expect(getHintOpacity(tester), 0.0);
        });

        testWidgets('hint has correct style when visible', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isEmpty: true,
              decoration: const InputDecoration(border: OutlineInputBorder(), hintText: hintText),
            ),
          );

          // Hint is visible because decorator is empty.
          expect(getHintOpacity(tester), 1.0);

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          final TextStyle expectedStyle = theme.textTheme.bodyLarge!.copyWith(color: expectedColor);
          expect(getHintStyle(tester), expectedStyle);
        });
      });

      group('when field is disabled', () {
        testWidgets('hint and input align vertically when decorator is empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isEmpty: true,
              decoration: const InputDecoration(
                enabled: false,
                border: OutlineInputBorder(),
                hintText: hintText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 16.0);
          expect(getInputRect(tester).bottom, 40.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is visible because decorator is empty.
          expect(getHintOpacity(tester), 1.0);
        });

        testWidgets('hint and input align vertically when decorator is not empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                enabled: false,
                border: OutlineInputBorder(),
                hintText: hintText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 16.0);
          expect(getInputRect(tester).bottom, 40.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not empty.
          expect(getHintOpacity(tester), 0.0);
        });

        testWidgets('hint has correct style when visible', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isEmpty: true,
              decoration: const InputDecoration(
                enabled: false,
                border: OutlineInputBorder(),
                hintText: hintText,
              ),
            ),
          );

          // Hint is visible because decorator is empty.
          expect(getHintOpacity(tester), 1.0);

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurface.withOpacity(0.38);
          final TextStyle expectedStyle = theme.textTheme.bodyLarge!.copyWith(color: expectedColor);
          expect(getHintStyle(tester), expectedStyle);
        });
      });

      group('when field is hovered', () {
        testWidgets('hint and input align vertically when decorator is empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              isEmpty: true,
              decoration: const InputDecoration(border: OutlineInputBorder(), hintText: hintText),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 16.0);
          expect(getInputRect(tester).bottom, 40.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is visible because decorator is empty.
          expect(getHintOpacity(tester), 1.0);
        });

        testWidgets('hint and input align vertically when decorator is not empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(border: OutlineInputBorder(), hintText: hintText),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 16.0);
          expect(getInputRect(tester).bottom, 40.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not empty.
          expect(getHintOpacity(tester), 0.0);
        });

        testWidgets('hint has correct style when visible', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              isEmpty: true,
              decoration: const InputDecoration(border: OutlineInputBorder(), hintText: hintText),
            ),
          );

          // Hint is visible because decorator is empty.
          expect(getHintOpacity(tester), 1.0);

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          final TextStyle expectedStyle = theme.textTheme.bodyLarge!.copyWith(color: expectedColor);
          expect(getHintStyle(tester), expectedStyle);
        });
      });

      group('when field is focused', () {
        testWidgets('hint and input align vertically when decorator is empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              isEmpty: true,
              decoration: const InputDecoration(border: OutlineInputBorder(), hintText: hintText),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 16.0);
          expect(getInputRect(tester).bottom, 40.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is visible because decorator is empty.
          expect(getHintOpacity(tester), 1.0);
        });

        testWidgets('hint and input align vertically when decorator is not empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(border: OutlineInputBorder(), hintText: hintText),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 16.0);
          expect(getInputRect(tester).bottom, 40.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not empty.
          expect(getHintOpacity(tester), 0.0);
        });

        testWidgets('hint has correct style when visible', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              isEmpty: true,
              decoration: const InputDecoration(border: OutlineInputBorder(), hintText: hintText),
            ),
          );

          // Hint is visible because decorator is empty.
          expect(getHintOpacity(tester), 1.0);

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          final TextStyle expectedStyle = theme.textTheme.bodyLarge!.copyWith(color: expectedColor);
          expect(getHintStyle(tester), expectedStyle);
        });
      });

      group('when field is in error', () {
        testWidgets('hint and input align vertically when decorator is empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isEmpty: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                hintText: hintText,
                errorText: errorText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 16.0);
          expect(getInputRect(tester).bottom, 40.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is visible because decorator is empty.
          expect(getHintOpacity(tester), 1.0);
        });

        testWidgets('hint and input align vertically when decorator is not empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                hintText: hintText,
                errorText: errorText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 16.0);
          expect(getInputRect(tester).bottom, 40.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not empty.
          expect(getHintOpacity(tester), 0.0);
        });

        testWidgets('hint has correct style when visible', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isEmpty: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                hintText: hintText,
                errorText: errorText,
              ),
            ),
          );

          // Hint is visible because decorator is empty.
          expect(getHintOpacity(tester), 1.0);

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          final TextStyle expectedStyle = theme.textTheme.bodyLarge!.copyWith(color: expectedColor);
          expect(getHintStyle(tester), expectedStyle);
        });
      });
    });

    group('for outlined text field with label', () {
      // Overall height for this InputDecorator is 56dp on mobile:
      //   16 - Top padding
      //   24 - Input and hint (font size = 16, line height = 1.5)
      //   16 - Bottom padding
      group('when field is enabled', () {
        testWidgets('hint and input align vertically when decorator is empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isEmpty: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                hintText: hintText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 16.0);
          expect(getInputRect(tester).bottom, 40.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not focused (label is visible).
          expect(getHintOpacity(tester), 0.0);
        });

        testWidgets('hint and input align vertically when decorator is not empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                hintText: hintText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 16.0);
          expect(getInputRect(tester).bottom, 40.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not empty.
          expect(getHintOpacity(tester), 0.0);
        });
      });

      group('when field is disabled', () {
        testWidgets('hint and input align vertically when decorator is empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isEmpty: true,
              decoration: const InputDecoration(
                enabled: false,
                border: OutlineInputBorder(),
                labelText: labelText,
                hintText: hintText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 16.0);
          expect(getInputRect(tester).bottom, 40.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not focused (label is visible).
          expect(getHintOpacity(tester), 0.0);
        });

        testWidgets('hint and input align vertically when decorator is not empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                enabled: false,
                border: OutlineInputBorder(),
                labelText: labelText,
                hintText: hintText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 16.0);
          expect(getInputRect(tester).bottom, 40.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not empty.
          expect(getHintOpacity(tester), 0.0);
        });
      });

      group('when field is hovered', () {
        testWidgets('hint and input align vertically when decorator is empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              isEmpty: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                hintText: hintText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 16.0);
          expect(getInputRect(tester).bottom, 40.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not focused (label is visible).
          expect(getHintOpacity(tester), 0.0);
        });

        testWidgets('hint and input align vertically when decorator is not empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                hintText: hintText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 16.0);
          expect(getInputRect(tester).bottom, 40.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not empty.
          expect(getHintOpacity(tester), 0.0);
        });
      });

      group('when field is focused', () {
        testWidgets('hint and input align vertically when decorator is empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              isEmpty: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                hintText: hintText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 16.0);
          expect(getInputRect(tester).bottom, 40.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is visible because decorator is empty.
          expect(getHintOpacity(tester), 1.0);
        });

        testWidgets('hint and input align vertically when decorator is not empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                hintText: hintText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 16.0);
          expect(getInputRect(tester).bottom, 40.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not empty.
          expect(getHintOpacity(tester), 0.0);
        });

        testWidgets('hint has correct style when visible', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              isEmpty: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                hintText: hintText,
              ),
            ),
          );

          // Hint is visible because decorator is empty.
          expect(getHintOpacity(tester), 1.0);

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          final TextStyle expectedStyle = theme.textTheme.bodyLarge!.copyWith(color: expectedColor);
          expect(getHintStyle(tester), expectedStyle);
        });
      });

      group('when field is in error', () {
        testWidgets('hint and input align vertically when decorator is empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isEmpty: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                hintText: hintText,
                errorText: errorText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 16.0);
          expect(getInputRect(tester).bottom, 40.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not focused (label is visible).
          expect(getHintOpacity(tester), 0.0);
        });

        testWidgets('hint and input align vertically when decorator is not empty', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                hintText: hintText,
                errorText: errorText,
              ),
            ),
          );

          expect(getContainerRect(tester).height, 56.0);
          expect(getInputRect(tester).top, 16.0);
          expect(getInputRect(tester).bottom, 40.0);
          expect(getHintRect(tester).top, getInputRect(tester).top);
          expect(getHintRect(tester).bottom, getInputRect(tester).bottom);
          // Hint is not visible because decorator is not empty.
          expect(getHintOpacity(tester), 0.0);
        });
      });
    });

    group('InputDecoration.alignLabelWithHint', () {
      testWidgets('positions InputDecoration.labelText vertically aligned with the hint', (
        WidgetTester tester,
      ) async {
        await tester.pumpWidget(
          buildInputDecorator(
            isEmpty: true,
            decoration: const InputDecoration(
              labelText: labelText,
              alignLabelWithHint: true,
              hintText: hintText,
            ),
          ),
        );

        // Label and hint should be vertically aligned.
        expect(getLabelCenter(tester).dy, getHintCenter(tester).dy);
      });

      testWidgets('positions InputDecoration.label vertically aligned with the hint', (
        WidgetTester tester,
      ) async {
        await tester.pumpWidget(
          buildInputDecorator(
            isEmpty: true,
            decoration: const InputDecoration(
              label: customLabel,
              alignLabelWithHint: true,
              hintText: hintText,
            ),
          ),
        );

        // Label and hint should be vertically aligned.
        expect(getCustomLabelCenter(tester).dy, getHintCenter(tester).dy);
      });

      group('in non-expanded multiline TextField', () {
        testWidgets('positions the label correctly when strut is disabled', (
          WidgetTester tester,
        ) async {
          final FocusNode focusNode = FocusNode();
          final TextEditingController controller = TextEditingController();
          addTearDown(() {
            focusNode.dispose();
            controller.dispose();
          });

          Widget buildFrame(bool alignLabelWithHint) {
            return MaterialApp(
              home: Material(
                child: Align(
                  alignment: Alignment.topLeft,
                  child: Directionality(
                    textDirection: TextDirection.ltr,
                    child: TextField(
                      controller: controller,
                      focusNode: focusNode,
                      maxLines: 8,
                      decoration: InputDecoration(
                        labelText: labelText,
                        alignLabelWithHint: alignLabelWithHint,
                        hintText: hintText,
                      ),
                      strutStyle: StrutStyle.disabled,
                    ),
                  ),
                ),
              ),
            );
          }

          // `alignLabelWithHint: false` centers the label vertically in the TextField.
          await tester.pumpWidget(buildFrame(false));
          await tester.pump(kTransitionDuration);
          expect(getLabelCenter(tester).dy, getDecoratorCenter(tester).dy);

          // Entering text still happens at the top.
          await tester.enterText(find.byType(TextField), inputText);
          expect(getInputRect(tester).top, 24.0);
          controller.clear();
          focusNode.unfocus();

          // `alignLabelWithHint: true` aligns the label vertically with the hint.
          await tester.pumpWidget(buildFrame(true));
          await tester.pump(kTransitionDuration);
          expect(getLabelCenter(tester).dy, getHintCenter(tester).dy);

          // Entering text still happens at the top.
          await tester.enterText(find.byType(TextField), inputText);
          expect(getInputRect(tester).top, 24.0);
          controller.clear();
          focusNode.unfocus();
        });

        testWidgets('positions the label correctly when strut style is set to default', (
          WidgetTester tester,
        ) async {
          final FocusNode focusNode = FocusNode();
          final TextEditingController controller = TextEditingController();
          addTearDown(() {
            focusNode.dispose();
            controller.dispose();
          });

          Widget buildFrame(bool alignLabelWithHint) {
            return MaterialApp(
              home: Material(
                child: Align(
                  alignment: Alignment.topLeft,
                  child: Directionality(
                    textDirection: TextDirection.ltr,
                    child: TextField(
                      controller: controller,
                      focusNode: focusNode,
                      maxLines: 8,
                      decoration: InputDecoration(
                        labelText: labelText,
                        alignLabelWithHint: alignLabelWithHint,
                        hintText: hintText,
                      ),
                    ),
                  ),
                ),
              ),
            );
          }

          // `alignLabelWithHint: false` centers the label vertically in the TextField.
          await tester.pumpWidget(buildFrame(false));
          await tester.pump(kTransitionDuration);
          expect(getLabelCenter(tester).dy, getDecoratorCenter(tester).dy);

          // Entering text still happens at the top.
          await tester.enterText(find.byType(InputDecorator), inputText);
          expect(getInputRect(tester).top, 24.0);
          controller.clear();
          focusNode.unfocus();

          // `alignLabelWithHint: true` aligns the label vertically with the hint.
          await tester.pumpWidget(buildFrame(true));
          await tester.pump(kTransitionDuration);
          expect(getLabelCenter(tester).dy, getHintCenter(tester).dy);

          // Entering text still happens at the top.
          await tester.enterText(find.byType(InputDecorator), inputText);
          expect(getInputRect(tester).top, 24.0);
          controller.clear();
          focusNode.unfocus();
        });
      });

      group('in expanded multiline TextField', () {
        testWidgets('positions the label correctly', (WidgetTester tester) async {
          final FocusNode focusNode = FocusNode();
          final TextEditingController controller = TextEditingController();
          addTearDown(() {
            focusNode.dispose();
            controller.dispose();
          });

          Widget buildFrame(bool alignLabelWithHint) {
            return MaterialApp(
              home: Material(
                child: Align(
                  alignment: Alignment.topLeft,
                  child: Directionality(
                    textDirection: TextDirection.ltr,
                    child: TextField(
                      controller: controller,
                      focusNode: focusNode,
                      maxLines: null,
                      expands: true,
                      decoration: InputDecoration(
                        labelText: labelText,
                        alignLabelWithHint: alignLabelWithHint,
                        hintText: hintText,
                      ),
                    ),
                  ),
                ),
              ),
            );
          }

          // `alignLabelWithHint: false` centers the label vertically in the TextField.
          await tester.pumpWidget(buildFrame(false));
          await tester.pump(kTransitionDuration);
          expect(getLabelCenter(tester).dy, getDecoratorCenter(tester).dy);

          // Entering text still happens at the top.
          await tester.enterText(find.byType(InputDecorator), inputText);
          expect(getInputRect(tester).top, 24.0);
          controller.clear();
          focusNode.unfocus();

          // alignLabelWithHint: true aligns the label vertically with the hint at the top.
          await tester.pumpWidget(buildFrame(true));
          await tester.pump(kTransitionDuration);
          expect(getLabelCenter(tester).dy, getHintCenter(tester).dy);

          // Entering text still happens at the top.
          await tester.enterText(find.byType(InputDecorator), inputText);
          expect(getInputRect(tester).top, 24.0);
          controller.clear();
          focusNode.unfocus();
        });

        testWidgets('positions the label correctly when border is outlined', (
          WidgetTester tester,
        ) async {
          final FocusNode focusNode = FocusNode();
          final TextEditingController controller = TextEditingController();
          addTearDown(() {
            focusNode.dispose();
            controller.dispose();
          });

          Widget buildFrame(bool alignLabelWithHint) {
            return MaterialApp(
              home: Material(
                child: Align(
                  alignment: Alignment.topLeft,
                  child: Directionality(
                    textDirection: TextDirection.ltr,
                    child: TextField(
                      controller: controller,
                      focusNode: focusNode,
                      maxLines: null,
                      expands: true,
                      decoration: InputDecoration(
                        labelText: labelText,
                        alignLabelWithHint: alignLabelWithHint,
                        hintText: hintText,
                        border: const OutlineInputBorder(borderRadius: BorderRadius.zero),
                      ),
                    ),
                  ),
                ),
              ),
            );
          }

          // `alignLabelWithHint: false` centers the label vertically in the TextField.
          await tester.pumpWidget(buildFrame(false));
          await tester.pump(kTransitionDuration);
          expect(getLabelCenter(tester).dy, getDecoratorCenter(tester).dy);

          // Entering text happens in the center as well.
          await tester.enterText(find.byType(InputDecorator), inputText);
          expect(getInputCenter(tester).dy, getDecoratorCenter(tester).dy);
          controller.clear();
          focusNode.unfocus();

          // `alignLabelWithHint: true` aligns keeps the label in the center because
          // that's where the hint is.
          await tester.pumpWidget(buildFrame(true));
          await tester.pump(kTransitionDuration);

          // On M3, hint centering is slightly wrong.
          // TODO(bleroux): remove closeTo usage when this is fixed.
          expect(getHintCenter(tester).dy, closeTo(getDecoratorCenter(tester).dy, 2.0));
          expect(getLabelCenter(tester).dy, getHintCenter(tester).dy);

          // Entering text still happens in the center.
          await tester.enterText(find.byType(InputDecorator), inputText);
          expect(getInputCenter(tester).dy, getDecoratorCenter(tester).dy);
          controller.clear();
          focusNode.unfocus();
        });
      });

      group('Horizontal alignment', () {
        testWidgets('Label for outlined decoration aligns horizontally with prefixIcon by default', (
          WidgetTester tester,
        ) async {
          // Regression test for https://github.com/flutter/flutter/issues/113537.
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                prefixIcon: Icon(Icons.ac_unit),
                labelText: labelText,
                border: OutlineInputBorder(),
              ),
              isFocused: true,
            ),
          );

          // Label left padding is 16.0 (12.0 right padding for a decoration with icons + 4.0 extra padding for the floating label)
          expect(getLabelRect(tester).left, 16.0);
          // Based on M3 spec, the expected horizontal position is 52 (12 padding, 24 icon, 16 gap between icon and input).
          // See https://m3.material.io/components/text-fields/specs#1ad2798c-ab41-4f0c-9a97-295ab9b37f33
          // (Note that the diagrams on the spec for outlined text field are wrong but the table for
          // outlined text fields and the diagrams for filled text field point to these values).
          expect(getInputRect(tester).left, 52.0);
        });

        testWidgets(
          'Label for outlined decoration aligns horizontally with input when alignLabelWithHint is true',
          (WidgetTester tester) async {
            // Regression test for https://github.com/flutter/flutter/issues/113537.
            await tester.pumpWidget(
              buildInputDecorator(
                decoration: const InputDecoration(
                  prefixIcon: Icon(Icons.ac_unit),
                  labelText: labelText,
                  border: OutlineInputBorder(),
                  alignLabelWithHint: true,
                ),
                isFocused: true,
              ),
            );

            expect(getLabelRect(tester).left, getInputRect(tester).left);
          },
        );

        testWidgets('Label for filled decoration is horizontally aligned with text by default', (
          WidgetTester tester,
        ) async {
          // Regression test for https://github.com/flutter/flutter/issues/113537.
          // See https://github.com/flutter/flutter/pull/115540.
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                prefixIcon: Icon(Icons.ac_unit),
                labelText: labelText,
                filled: true,
              ),
              isFocused: true,
            ),
          );

          // Label and input are horizontally aligned despite `alignLabelWithHint` being false (default value).
          // The reason is that `alignLabelWithHint` was initially intended for vertical alignment only.
          // See https://github.com/flutter/flutter/pull/24993 which introduced `alignLabelWithHint` parameter.
          // See https://github.com/flutter/flutter/pull/115409 which used `alignLabelWithHint` for
          // horizontal alignment in outlined text field.
          expect(getLabelRect(tester).left, getInputRect(tester).left);
        });
      });
    });

    group('hint opacity animation', () {
      testWidgets('default duration', (WidgetTester tester) async {
        // Build once without focus.
        await tester.pumpWidget(
          buildInputDecorator(
            isEmpty: true,
            decoration: const InputDecoration(labelText: labelText, hintText: hintText),
          ),
        );

        // Hint is not visible (opacity 0.0).
        expect(getHintOpacity(tester), 0.0);

        // Focus the decorator to trigger the animation.
        await tester.pumpWidget(
          buildInputDecorator(
            isEmpty: true,
            isFocused: true,
            decoration: const InputDecoration(labelText: labelText, hintText: hintText),
          ),
        );

        // The hint's opacity animates from 0.0 to 1.0.
        // The animation's default duration is 20ms.
        await tester.pump(const Duration(milliseconds: 9));
        double hintOpacity9ms = getHintOpacity(tester);
        expect(hintOpacity9ms, inExclusiveRange(0.0, 1.0));
        await tester.pump(const Duration(milliseconds: 9));
        double hintOpacity18ms = getHintOpacity(tester);
        expect(hintOpacity18ms, inExclusiveRange(hintOpacity9ms, 1.0));

        await tester.pump(kTransitionDuration);
        // Hint is fully visible (opacity 1.0).
        expect(getHintOpacity(tester), 1.0);

        // Unfocus the decorator to trigger the reversed animation.
        await tester.pumpWidget(
          buildInputDecorator(
            isEmpty: true,
            decoration: const InputDecoration(labelText: labelText, hintText: hintText),
          ),
        );

        // The hint's opacity animates from 1.0 to 0.0.
        // The animation's default duration is 20ms.
        await tester.pump(const Duration(milliseconds: 9));
        hintOpacity9ms = getHintOpacity(tester);
        expect(hintOpacity9ms, inExclusiveRange(0.0, 1.0));
        await tester.pump(const Duration(milliseconds: 9));
        hintOpacity18ms = getHintOpacity(tester);
        expect(hintOpacity18ms, inExclusiveRange(0.0, hintOpacity9ms));
      });

      testWidgets('custom duration', (WidgetTester tester) async {
        // Build once without focus.
        await tester.pumpWidget(
          buildInputDecorator(
            isEmpty: true,
            decoration: const InputDecoration(
              labelText: labelText,
              hintText: hintText,
              hintFadeDuration: Duration(milliseconds: 120),
            ),
          ),
        );

        // Hint is not visible (opacity 0.0).
        expect(getHintOpacity(tester), 0.0);

        // Focus the decorator to trigger the animation.
        await tester.pumpWidget(
          buildInputDecorator(
            isEmpty: true,
            isFocused: true,
            decoration: const InputDecoration(
              labelText: labelText,
              hintText: hintText,
              hintFadeDuration: Duration(milliseconds: 120),
            ),
          ),
        );

        // The hint's opacity animates from 0.0 to 1.0.
        // The animation's duration is set to 120ms.
        await tester.pump(const Duration(milliseconds: 50));
        double hintOpacity50ms = getHintOpacity(tester);
        expect(hintOpacity50ms, inExclusiveRange(0.0, 1.0));
        await tester.pump(const Duration(milliseconds: 50));
        double hintOpacity100ms = getHintOpacity(tester);
        expect(hintOpacity100ms, inExclusiveRange(hintOpacity50ms, 1.0));
        await tester.pump(const Duration(milliseconds: 50));
        expect(getHintOpacity(tester), 1.0);

        // Unfocus the decorator to trigger the reversed animation.
        await tester.pumpWidget(
          buildInputDecorator(
            isEmpty: true,
            decoration: const InputDecoration(
              labelText: labelText,
              hintText: hintText,
              hintFadeDuration: Duration(milliseconds: 120),
            ),
          ),
        );

        // The hint's opacity animates from 1.0 to 0.0.
        // The animation's default duration is 20ms.
        await tester.pump(const Duration(milliseconds: 50));
        hintOpacity50ms = getHintOpacity(tester);
        expect(hintOpacity50ms, inExclusiveRange(0.0, 1.0));
        await tester.pump(const Duration(milliseconds: 50));
        hintOpacity100ms = getHintOpacity(tester);
        expect(hintOpacity100ms, inExclusiveRange(0.0, hintOpacity50ms));
        await tester.pump(const Duration(milliseconds: 50));
        expect(getHintOpacity(tester), 0.0);
      });

      testWidgets('duration from theme', (WidgetTester tester) async {
        // Build once without focus.
        await tester.pumpWidget(
          buildInputDecorator(
            isEmpty: true,
            decoration: const InputDecoration(labelText: labelText, hintText: hintText),
            inputDecorationTheme: const InputDecorationTheme(
              hintFadeDuration: Duration(milliseconds: 120),
            ),
          ),
        );

        // Hint is not visible (opacity 0.0).
        expect(getHintOpacity(tester), 0.0);

        // Focus the decorator to trigger the animation.
        await tester.pumpWidget(
          buildInputDecorator(
            isEmpty: true,
            isFocused: true,
            decoration: const InputDecoration(labelText: labelText, hintText: hintText),
            inputDecorationTheme: const InputDecorationTheme(
              hintFadeDuration: Duration(milliseconds: 120),
            ),
          ),
        );

        // The hint's opacity animates from 0.0 to 1.0.
        // The animation's duration is set to 120ms.
        await tester.pump(const Duration(milliseconds: 50));
        double hintOpacity50ms = getHintOpacity(tester);
        expect(hintOpacity50ms, inExclusiveRange(0.0, 1.0));
        await tester.pump(const Duration(milliseconds: 50));
        double hintOpacity100ms = getHintOpacity(tester);
        expect(hintOpacity100ms, inExclusiveRange(hintOpacity50ms, 1.0));
        await tester.pump(const Duration(milliseconds: 50));
        expect(getHintOpacity(tester), 1.0);

        // Unfocus the decorator to trigger the reversed animation.
        await tester.pumpWidget(
          buildInputDecorator(
            isEmpty: true,
            decoration: const InputDecoration(labelText: labelText, hintText: hintText),
            inputDecorationTheme: const InputDecorationTheme(
              hintFadeDuration: Duration(milliseconds: 120),
            ),
          ),
        );

        // The hint's opacity animates from 1.0 to 0.0.
        // The animation's default duration is 20ms.
        await tester.pump(const Duration(milliseconds: 50));
        hintOpacity50ms = getHintOpacity(tester);
        expect(hintOpacity50ms, inExclusiveRange(0.0, 1.0));
        await tester.pump(const Duration(milliseconds: 50));
        hintOpacity100ms = getHintOpacity(tester);
        expect(hintOpacity100ms, inExclusiveRange(0.0, hintOpacity50ms));
        await tester.pump(const Duration(milliseconds: 50));
        expect(getHintOpacity(tester), 0.0);
      });
    });

    testWidgets('InputDecorator throws Assertion Error when hint and hintText are provided', (
      WidgetTester tester,
    ) async {
      expect(() {
        buildInputDecorator(
          decoration: InputDecoration(
            hintText: 'Enter text here',
            hint: const Text('Enter text here', style: TextStyle(fontSize: 20.0)),
          ),
        );
      }, throwsAssertionError);
    });

    testWidgets('InputDecorator shows hint widget', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecorator(decoration: const InputDecoration(hint: Text('hint'))),
      );
      expect(find.text('hint'), findsOneWidget);
    });

    testWidgets('hint style overflow works', (WidgetTester tester) async {
      final String hintText = 'hint text' * 20;
      const TextStyle hintStyle = TextStyle(fontSize: 14.0, overflow: TextOverflow.fade);
      final InputDecoration decoration = InputDecoration(hintText: hintText, hintStyle: hintStyle);

      await tester.pumpWidget(buildInputDecorator(decoration: decoration));
      await tester.pump(kTransitionDuration);

      final Finder hintTextFinder = find.text(hintText);
      final Text hintTextWidget = tester.widget(hintTextFinder);
      expect(hintTextWidget.style!.overflow, decoration.hintStyle!.overflow);
    });

    testWidgets('Widget height collapses from hint height when maintainHintSize is false', (
      WidgetTester tester,
    ) async {
      final String hintText = 'hint' * 20;
      final InputDecoration decoration = InputDecoration(
        hintText: hintText,
        hintMaxLines: 3,
        maintainHintSize: false,
      );

      await tester.pumpWidget(buildInputDecorator(decoration: decoration));
      expect(tester.getSize(find.byType(InputDecorator)).height, 48.0);
    });

    testWidgets('Widget height stays at hint height by default', (WidgetTester tester) async {
      final String hintText = 'hint' * 20;
      final InputDecoration decoration = InputDecoration(hintMaxLines: 3, hintText: hintText);

      await tester.pumpWidget(buildInputDecorator(decoration: decoration));
      final double hintHeight = tester.getSize(find.text(hintText)).height;
      final double inputHeight = tester.getSize(find.byType(InputDecorator)).height;
      expect(inputHeight, hintHeight + 16.0);
    });

    testWidgets('hintFadeDuration applies to hint fade-in when maintainHintSize is false', (
      WidgetTester tester,
    ) async {
      const InputDecoration decoration = InputDecoration(
        hintText: hintText,
        hintMaxLines: 3,
        hintFadeDuration: Duration(milliseconds: 120),
        maintainHintSize: false,
      );

      // Build once with empty content.
      await tester.pumpWidget(buildInputDecorator(decoration: decoration));

      // Hint is not exist.
      expect(find.text(hintText), findsNothing);

      // Rebuild with empty content.
      await tester.pumpWidget(buildInputDecorator(isEmpty: true, decoration: decoration));

      // The hint's opacity animates from 0.0 to 1.0.
      // The animation's default duration is 20ms.
      await tester.pump(const Duration(milliseconds: 50));
      final double hintOpacity50ms = getHintOpacity(tester);
      expect(hintOpacity50ms, inExclusiveRange(0.0, 1.0));
      await tester.pump(const Duration(milliseconds: 50));
      final double hintOpacity100ms = getHintOpacity(tester);
      expect(hintOpacity100ms, inExclusiveRange(hintOpacity50ms, 1.0));
      await tester.pump(const Duration(milliseconds: 20));
      final double hintOpacity120ms = getHintOpacity(tester);
      expect(hintOpacity120ms, 1.0);
    });

    testWidgets('hintFadeDuration applies to hint fade-out when maintainHintSize is false', (
      WidgetTester tester,
    ) async {
      const InputDecoration decoration = InputDecoration(
        hintText: hintText,
        hintMaxLines: 3,
        hintFadeDuration: Duration(milliseconds: 120),
        maintainHintSize: false,
      );

      // Build once with empty content.
      await tester.pumpWidget(buildInputDecorator(isEmpty: true, decoration: decoration));

      // Hint is visible (opacity 1.0).
      expect(getHintOpacity(tester), 1.0);

      // Rebuild with non-empty content.
      await tester.pumpWidget(buildInputDecorator(decoration: decoration));

      // The hint's opacity animates from 1.0 to 0.0.
      // The animation's default duration is 20ms.
      await tester.pump(const Duration(milliseconds: 50));
      final double hintOpacity50ms = getHintOpacity(tester);
      expect(hintOpacity50ms, inExclusiveRange(0.0, 1.0));
      await tester.pump(const Duration(milliseconds: 50));
      final double hintOpacity100ms = getHintOpacity(tester);
      expect(hintOpacity100ms, inExclusiveRange(0.0, hintOpacity50ms));
      await tester.pump(const Duration(milliseconds: 20));
      final double hintOpacity120ms = getHintOpacity(tester);
      expect(hintOpacity120ms, 0);
      await tester.pump(const Duration(milliseconds: 1));
      // The hintText replaced with SizeBox.
      expect(find.text(hintText), findsNothing);
    });
  });

  group('Material3 - InputDecoration helper/counter/error', () {
    // Overall height for InputDecorator (filled or outlined) is 76dp on mobile:
    //    8 - top padding
    //   12 - floating label (font size = 16 * 0.75, line height is forced to 1.0)
    //    4 - gap between label and input
    //   24 - input text (font size = 16, line height = 1.5)
    //    8 - bottom padding
    //    4 - gap above helper/error/counter
    //   16 - helper/counter (font size = 12, line height is 1.5)
    const double topPadding = 8.0;
    const double floatingLabelHeight = 12.0;
    const double labelInputGap = 4.0;
    const double inputHeight = 24.0;
    const double bottomPadding = 8.0;
    const double helperGap = 4.0;
    const double helperHeight = 16.0;
    const double containerHeight =
        topPadding + floatingLabelHeight + labelInputGap + inputHeight + bottomPadding; // 56.0
    const double fullHeight = containerHeight + helperGap + helperHeight; // 76.0
    const double errorHeight = helperHeight;
    // TODO(bleroux): consider changing this padding because, from the M3 specification, it should be 16.
    const double helperStartPadding = 12.0;
    const double counterEndPadding = 12.0;

    group('for filled text field', () {
      group('when field is enabled', () {
        testWidgets('Helper and counter are correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
              ),
            ),
          );

          expect(getDecoratorRect(tester).height, fullHeight);
          expect(getBorderBottom(tester), containerHeight);
          expect(getHelperRect(tester).top, containerHeight + helperGap);
          expect(getHelperRect(tester).height, helperHeight);
          expect(getHelperRect(tester).left, helperStartPadding);
          expect(getCounterRect(tester).top, containerHeight + helperGap);
          expect(getCounterRect(tester).height, helperHeight);
          expect(getCounterRect(tester).right, 800 - counterEndPadding);
        });

        testWidgets('Helper and counter are correctly styled', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          final TextStyle expectedStyle = theme.textTheme.bodySmall!.copyWith(color: expectedColor);
          expect(getHelperStyle(tester), expectedStyle);
          expect(getCounterStyle(tester), expectedStyle);
        });
      });

      group('when field is disabled', () {
        testWidgets('Helper and counter are correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                enabled: false,
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
              ),
            ),
          );

          expect(getDecoratorRect(tester).height, fullHeight);
          expect(getBorderBottom(tester), containerHeight);
          expect(getHelperRect(tester).top, containerHeight + helperGap);
          expect(getHelperRect(tester).height, helperHeight);
          expect(getHelperRect(tester).left, helperStartPadding);
          expect(getCounterRect(tester).top, containerHeight + helperGap);
          expect(getCounterRect(tester).height, helperHeight);
          expect(getCounterRect(tester).right, 800 - counterEndPadding);
        });

        testWidgets('Helper and counter are correctly styled', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                enabled: false,
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurface.withOpacity(0.38);
          final TextStyle expectedStyle = theme.textTheme.bodySmall!.copyWith(color: expectedColor);
          expect(getHelperStyle(tester), expectedStyle);
          expect(getCounterStyle(tester), expectedStyle);
        });
      });

      group('when field is hovered', () {
        testWidgets('Helper and counter are correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
              ),
            ),
          );

          expect(getDecoratorRect(tester).height, fullHeight);
          expect(getBorderBottom(tester), containerHeight);
          expect(getHelperRect(tester).top, containerHeight + helperGap);
          expect(getHelperRect(tester).height, helperHeight);
          expect(getHelperRect(tester).left, helperStartPadding);
          expect(getCounterRect(tester).top, containerHeight + helperGap);
          expect(getCounterRect(tester).height, helperHeight);
          expect(getCounterRect(tester).right, 800 - counterEndPadding);
        });

        testWidgets('Helper and counter are correctly styled', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          final TextStyle expectedStyle = theme.textTheme.bodySmall!.copyWith(color: expectedColor);
          expect(getHelperStyle(tester), expectedStyle);
          expect(getCounterStyle(tester), expectedStyle);
        });
      });

      group('when field is focused', () {
        testWidgets('Helper and counter are correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
              ),
            ),
          );

          expect(getDecoratorRect(tester).height, fullHeight);
          expect(getBorderBottom(tester), containerHeight);
          expect(getHelperRect(tester).top, containerHeight + helperGap);
          expect(getHelperRect(tester).height, helperHeight);
          expect(getHelperRect(tester).left, helperStartPadding);
          expect(getCounterRect(tester).top, containerHeight + helperGap);
          expect(getCounterRect(tester).height, helperHeight);
          expect(getCounterRect(tester).right, 800 - counterEndPadding);
        });

        testWidgets('Helper and counter are correctly styled', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          final TextStyle expectedStyle = theme.textTheme.bodySmall!.copyWith(color: expectedColor);
          expect(getHelperStyle(tester), expectedStyle);
          expect(getCounterStyle(tester), expectedStyle);
        });
      });

      group('when field is in error', () {
        testWidgets('Error and counter are visible, helper is not visible', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
                errorText: errorText,
              ),
            ),
          );

          expect(findError(), findsOneWidget);
          expect(findCounter(), findsOneWidget);
          expect(findHelper(), findsNothing);
        });

        testWidgets('Error and counter are correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
                errorText: errorText,
              ),
            ),
          );

          expect(getDecoratorRect(tester).height, fullHeight);
          expect(getBorderBottom(tester), containerHeight);
          expect(getErrorRect(tester).top, containerHeight + helperGap);
          expect(getErrorRect(tester).height, errorHeight);
          expect(getErrorRect(tester).left, helperStartPadding);
          expect(getCounterRect(tester).top, containerHeight + helperGap);
          expect(getCounterRect(tester).height, errorHeight);
          expect(getCounterRect(tester).right, 800 - counterEndPadding);
        });

        testWidgets('Error and counter are correctly styled', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
                errorText: errorText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.error;
          final TextStyle expectedStyle = theme.textTheme.bodySmall!.copyWith(color: expectedColor);
          expect(getErrorStyle(tester), expectedStyle);
          final Color expectedCounterColor = theme.colorScheme.onSurfaceVariant;
          final TextStyle expectedCounterStyle = theme.textTheme.bodySmall!.copyWith(
            color: expectedCounterColor,
          );
          expect(getCounterStyle(tester), expectedCounterStyle);
        });
      });
    });

    group('for outlined text field', () {
      group('when field is enabled', () {
        testWidgets('Helper and counter are correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
              ),
            ),
          );

          expect(getDecoratorRect(tester).height, fullHeight);
          expect(getBorderBottom(tester), containerHeight);
          expect(getHelperRect(tester).top, containerHeight + helperGap);
          expect(getHelperRect(tester).height, helperHeight);
          expect(getHelperRect(tester).left, helperStartPadding);
          expect(getCounterRect(tester).top, containerHeight + helperGap);
          expect(getCounterRect(tester).height, helperHeight);
          expect(getCounterRect(tester).right, 800 - counterEndPadding);
        });

        testWidgets('Helper and counter are correctly styled', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          final TextStyle expectedStyle = theme.textTheme.bodySmall!.copyWith(color: expectedColor);
          expect(getHelperStyle(tester), expectedStyle);
          expect(getCounterStyle(tester), expectedStyle);
        });
      });

      group('when field is disabled', () {
        testWidgets('Helper and counter are correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                enabled: false,
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
              ),
            ),
          );

          expect(getDecoratorRect(tester).height, fullHeight);
          expect(getBorderBottom(tester), containerHeight);
          expect(getHelperRect(tester).top, containerHeight + helperGap);
          expect(getHelperRect(tester).height, helperHeight);
          expect(getHelperRect(tester).left, helperStartPadding);
          expect(getCounterRect(tester).top, containerHeight + helperGap);
          expect(getCounterRect(tester).height, helperHeight);
          expect(getCounterRect(tester).right, 800 - counterEndPadding);
        });

        testWidgets('Helper and counter are correctly styled', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                enabled: false,
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurface.withOpacity(0.38);
          final TextStyle expectedStyle = theme.textTheme.bodySmall!.copyWith(color: expectedColor);
          expect(getHelperStyle(tester), expectedStyle);
          expect(getCounterStyle(tester), expectedStyle);
        });
      });

      group('when field is hovered', () {
        testWidgets('Helper and counter are correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
              ),
            ),
          );

          expect(getDecoratorRect(tester).height, fullHeight);
          expect(getBorderBottom(tester), containerHeight);
          expect(getHelperRect(tester).top, containerHeight + helperGap);
          expect(getHelperRect(tester).height, helperHeight);
          expect(getHelperRect(tester).left, helperStartPadding);
          expect(getCounterRect(tester).top, containerHeight + helperGap);
          expect(getCounterRect(tester).height, helperHeight);
          expect(getCounterRect(tester).right, 800 - counterEndPadding);
        });

        testWidgets('Helper and counter are correctly styled', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          final TextStyle expectedStyle = theme.textTheme.bodySmall!.copyWith(color: expectedColor);
          expect(getHelperStyle(tester), expectedStyle);
          expect(getCounterStyle(tester), expectedStyle);
        });
      });

      group('when field is focused', () {
        testWidgets('Helper and counter are correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
              ),
            ),
          );

          expect(getDecoratorRect(tester).height, fullHeight);
          expect(getBorderBottom(tester), containerHeight);
          expect(getHelperRect(tester).top, containerHeight + helperGap);
          expect(getHelperRect(tester).height, helperHeight);
          expect(getHelperRect(tester).left, helperStartPadding);
          expect(getCounterRect(tester).top, containerHeight + helperGap);
          expect(getCounterRect(tester).height, helperHeight);
          expect(getCounterRect(tester).right, 800 - counterEndPadding);
        });

        testWidgets('Helper and counter are correctly styled', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          final TextStyle expectedStyle = theme.textTheme.bodySmall!.copyWith(color: expectedColor);
          expect(getHelperStyle(tester), expectedStyle);
          expect(getCounterStyle(tester), expectedStyle);
        });
      });

      group('when field is in error', () {
        testWidgets('Error and counter are visible, helper is not visible', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
                errorText: errorText,
              ),
            ),
          );

          expect(findHelper(), findsNothing);
          expect(findError(), findsOneWidget);
          expect(findCounter(), findsOneWidget);
        });

        testWidgets('Error and counter are correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
                errorText: errorText,
              ),
            ),
          );

          expect(getDecoratorRect(tester).height, fullHeight);
          expect(getBorderBottom(tester), containerHeight);
          expect(getErrorRect(tester).top, containerHeight + helperGap);
          expect(getErrorRect(tester).height, errorHeight);
          expect(getErrorRect(tester).left, helperStartPadding);
          expect(getCounterRect(tester).top, containerHeight + helperGap);
          expect(getCounterRect(tester).height, errorHeight);
          expect(getCounterRect(tester).right, 800 - counterEndPadding);
        });

        testWidgets('Error and counter are correctly styled', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                helperText: helperText,
                counterText: counterText,
                errorText: errorText,
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findDecorator()));
          final Color expectedColor = theme.colorScheme.error;
          final TextStyle expectedStyle = theme.textTheme.bodySmall!.copyWith(color: expectedColor);
          expect(getErrorStyle(tester), expectedStyle);
          final Color expectedCounterColor = theme.colorScheme.onSurfaceVariant;
          final TextStyle expectedCounterStyle = theme.textTheme.bodySmall!.copyWith(
            color: expectedCounterColor,
          );
          expect(getCounterStyle(tester), expectedCounterStyle);
        });
      });
    });

    group('Multiline error/helper', () {
      testWidgets('Error height grows to accommodate error text', (WidgetTester tester) async {
        const int maxLines = 3;
        await tester.pumpWidget(
          buildInputDecorator(
            decoration: const InputDecoration(
              labelText: 'label',
              errorText: threeLines,
              errorMaxLines: maxLines,
              filled: true,
            ),
          ),
        );

        final Rect errorRect = tester.getRect(find.text(threeLines));
        expect(errorRect.height, closeTo(errorHeight * maxLines, 0.25));
        expect(
          getDecoratorRect(tester).height,
          closeTo(containerHeight + helperGap + errorHeight * maxLines, 0.25),
        );
      });

      testWidgets('Error height is correct when errorMaxLines is restricted', (
        WidgetTester tester,
      ) async {
        const int maxLines = 2;
        await tester.pumpWidget(
          buildInputDecorator(
            decoration: const InputDecoration(
              labelText: 'label',
              errorText: threeLines,
              errorMaxLines: maxLines,
              filled: true,
            ),
          ),
        );

        final Rect errorRect = tester.getRect(find.text(threeLines));
        expect(errorRect.height, closeTo(errorHeight * maxLines, 0.25));
        expect(
          getDecoratorRect(tester).height,
          closeTo(containerHeight + helperGap + errorHeight * maxLines, 0.25),
        );
      });

      testWidgets(
        'Error height is correct when errorMaxLines is bigger than the number of lines in errorText',
        (WidgetTester tester) async {
          const int numberOfLines = 2;
          const int maxLines = 3;
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                labelText: 'label',
                errorText: twoLines,
                errorMaxLines: maxLines,
                filled: true,
              ),
            ),
          );

          final Rect errorRect = tester.getRect(find.text(twoLines));
          expect(errorRect.height, closeTo(errorHeight * numberOfLines, 0.25));
          expect(
            getDecoratorRect(tester).height,
            closeTo(containerHeight + helperGap + errorHeight * numberOfLines, 0.25),
          );
        },
      );

      testWidgets('Error height is not limited by default', (WidgetTester tester) async {
        const int numberOfLines = 3;
        await tester.pumpWidget(
          buildInputDecorator(
            decoration: const InputDecoration(
              labelText: 'label',
              errorText: threeLines,
              filled: true,
            ),
          ),
        );

        final Rect errorRect = tester.getRect(find.text(threeLines));
        expect(errorRect.height, closeTo(errorHeight * numberOfLines, 0.25));
        expect(
          getDecoratorRect(tester).height,
          closeTo(containerHeight + helperGap + errorHeight * numberOfLines, 0.25),
        );
      });

      testWidgets('Helper height grows to accommodate helper text', (WidgetTester tester) async {
        const int maxLines = 3;
        await tester.pumpWidget(
          buildInputDecorator(
            decoration: const InputDecoration(
              labelText: 'label',
              helperText: threeLines,
              helperMaxLines: maxLines,
              filled: true,
            ),
          ),
        );

        final Rect helperRect = tester.getRect(find.text(threeLines));
        expect(helperRect.height, closeTo(helperHeight * maxLines, 0.25));
        expect(
          getDecoratorRect(tester).height,
          closeTo(containerHeight + helperGap + helperHeight * maxLines, 0.25),
        );
      });

      testWidgets('Helper height is correct when maxLines is restricted', (
        WidgetTester tester,
      ) async {
        const int maxLines = 2;
        await tester.pumpWidget(
          buildInputDecorator(
            decoration: const InputDecoration(
              labelText: 'label',
              helperText: threeLines,
              helperMaxLines: maxLines,
              filled: true,
            ),
          ),
        );

        final Rect helperRect = tester.getRect(find.text(threeLines));
        expect(helperRect.height, closeTo(helperHeight * maxLines, 0.25));
        expect(
          getDecoratorRect(tester).height,
          closeTo(containerHeight + helperGap + helperHeight * maxLines, 0.25),
        );
      });

      testWidgets(
        'Helper height is correct when helperMaxLines is bigger than the number of lines in helperText',
        (WidgetTester tester) async {
          const int numberOfLines = 2;
          const int maxLines = 3;
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                labelText: 'label',
                helperText: twoLines,
                helperMaxLines: maxLines,
                filled: true,
              ),
            ),
          );

          final Rect helperRect = tester.getRect(find.text(twoLines));
          expect(helperRect.height, closeTo(helperHeight * numberOfLines, 0.25));
          expect(
            getDecoratorRect(tester).height,
            closeTo(containerHeight + helperGap + helperHeight * numberOfLines, 0.25),
          );
        },
      );

      testWidgets('Helper height is not limited by default', (WidgetTester tester) async {
        const int numberOfLines = 3;
        await tester.pumpWidget(
          buildInputDecorator(
            decoration: const InputDecoration(
              labelText: 'label',
              helperText: threeLines,
              filled: true,
            ),
          ),
        );

        final Rect helperRect = tester.getRect(find.text(threeLines));
        expect(helperRect.height, closeTo(helperHeight * numberOfLines, 0.25));
        expect(
          getDecoratorRect(tester).height,
          closeTo(containerHeight + helperGap + helperHeight * numberOfLines, 0.25),
        );
      });
    });

    group('Helper widget', () {
      testWidgets('InputDecorator shows helper widget', (WidgetTester tester) async {
        await tester.pumpWidget(
          buildInputDecorator(
            decoration: const InputDecoration(
              helper: Text('helper', style: TextStyle(fontSize: 20.0)),
            ),
          ),
        );

        expect(find.text('helper'), findsOneWidget);
      });

      testWidgets('InputDecorator throws when helper text and helper widget are provided', (
        WidgetTester tester,
      ) async {
        expect(() {
          buildInputDecorator(
            decoration: InputDecoration(
              helperText: 'helperText',
              helper: const Text('helper', style: TextStyle(fontSize: 20.0)),
            ),
          );
        }, throwsAssertionError);
      });
    });

    group('Error widget', () {
      testWidgets('InputDecorator shows error widget', (WidgetTester tester) async {
        await tester.pumpWidget(
          buildInputDecorator(
            decoration: const InputDecoration(
              error: Text('error', style: TextStyle(fontSize: 20.0)),
            ),
          ),
        );

        expect(find.text('error'), findsOneWidget);
      });

      testWidgets('InputDecorator throws when error text and error widget are provided', (
        WidgetTester tester,
      ) async {
        expect(() {
          buildInputDecorator(
            decoration: InputDecoration(
              errorText: 'errorText',
              error: const Text('error', style: TextStyle(fontSize: 20.0)),
            ),
          );
        }, throwsAssertionError);
      });
    });

    testWidgets('InputDecorator with counter does not crash when given a 0 size', (
      WidgetTester tester,
    ) async {
      // Regression test for https://github.com/flutter/flutter/issues/129611
      await tester.pumpWidget(
        Center(
          child: SizedBox.square(
            dimension: 0.0,
            child: buildInputDecorator(
              decoration: const InputDecoration(
                contentPadding: EdgeInsetsDirectional.all(99),
                prefixIcon: Focus(child: Icon(Icons.search)),
                counter: Text('COUNTER'),
              ),
            ),
          ),
        ),
      );
      await tester.pump(kTransitionDuration);

      expect(find.byType(InputDecorator), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.text('COUNTER')).size, Size.zero);
    });
  });

  group('Material3 - InputDecoration constraints', () {
    testWidgets('No InputDecorator constraints', (WidgetTester tester) async {
      await tester.pumpWidget(buildInputDecorator());

      // Should fill the screen width and be default height.
      expect(getDecoratorRect(tester).size, const Size(800, kMinInteractiveDimension));
    });

    testWidgets('InputDecoratorThemeData constraints', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecorator(
          theme: ThemeData(
            inputDecorationTheme: const InputDecorationTheme(
              constraints: BoxConstraints(maxWidth: 300, maxHeight: 40),
            ),
          ),
        ),
      );

      // Theme settings should make it 300x40 pixels.
      expect(getDecoratorRect(tester).size, const Size(300, 40));
    });

    testWidgets('InputDecorator constraints', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecorator(
          theme: ThemeData(
            inputDecorationTheme: const InputDecorationTheme(
              constraints: BoxConstraints(maxWidth: 300, maxHeight: 40),
            ),
          ),
          decoration: const InputDecoration(
            constraints: BoxConstraints(maxWidth: 200, maxHeight: 32),
          ),
        ),
      );

      // InputDecoration.constraints should override the theme. It should be
      // only 200x32 pixels.
      expect(getDecoratorRect(tester).size, const Size(200, 32));
    });
  });

  group('Material3 - InputDecoration prefix/suffix', () {
    const IconData prefixIcon = Icons.search;
    const IconData suffixIcon = Icons.cancel_outlined;

    Finder findPrefixIcon() {
      return find.byIcon(prefixIcon);
    }

    Rect getPrefixIconRect(WidgetTester tester) {
      return tester.getRect(findPrefixIcon());
    }

    Finder findPrefixIconInnerRichText() {
      return find.descendant(of: findPrefixIcon(), matching: find.byType(RichText));
    }

    TextStyle getPrefixIconStyle(WidgetTester tester) {
      return tester.widget<RichText>(findPrefixIconInnerRichText()).text.style!;
    }

    Finder findSuffixIcon() {
      return find.byIcon(suffixIcon);
    }

    Rect getSuffixIconRect(WidgetTester tester) {
      return tester.getRect(findSuffixIcon());
    }

    Finder findSuffixIconInnerRichText() {
      return find.descendant(of: findSuffixIcon(), matching: find.byType(RichText));
    }

    TextStyle getSuffixIconStyle(WidgetTester tester) {
      return tester.widget<RichText>(findSuffixIconInnerRichText()).text.style!;
    }

    group('for filled text field', () {
      group('when field is enabled', () {
        testWidgets('prefixIcon is correctly positioned - LTR', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                prefixIcon: Icon(prefixIcon),
              ),
            ),
          );

          // By default, the prefix icon is rendered inside a 48x48 constrained box.
          expect(getPrefixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getPrefixIconStyle(tester).fontSize, 24.0);
          // Prefix icon is vertically centered inside the container.
          expect(getPrefixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Left padding is 12 per Material 3 spec.
          expect(tester.getRect(findPrefixIconInnerRichText()).left, 12.0);
          // Check the padding between the prefix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            getInputRect(tester).left - tester.getRect(findPrefixIconInnerRichText()).right,
            16.0,
          );
        });

        testWidgets('prefixIcon is correctly positioned - RTL', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              textDirection: TextDirection.rtl,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                prefixIcon: Icon(prefixIcon),
              ),
            ),
          );

          // By default, the prefix icon is rendered inside a 48x48 constrained box.
          expect(getPrefixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getPrefixIconStyle(tester).fontSize, 24.0);
          // Prefix icon is vertically centered inside the container.
          expect(getPrefixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Right padding is 12 per Material 3 spec.
          expect(
            getDecoratorRect(tester).right - tester.getRect(findPrefixIconInnerRichText()).right,
            12.0,
          );
          // Check the padding between the prefix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            tester.getRect(findPrefixIconInnerRichText()).left - getInputRect(tester).right,
            16.0,
          );
        });

        testWidgets('prefixIcon has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                prefixIcon: Icon(prefixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findPrefixIcon()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          expect(getPrefixIconStyle(tester).color, expectedColor);
        });

        testWidgets('suffixIcon is correctly positioned - LTR', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          // By default, the suffix icon is rendered inside a 48x48 constrained box.
          expect(getSuffixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getSuffixIconStyle(tester).fontSize, 24.0);
          // Suffix icon is vertically centered inside the container.
          expect(getSuffixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Right padding is 12 per Material 3 spec.
          expect(
            getDecoratorRect(tester).right - tester.getRect(findSuffixIconInnerRichText()).right,
            12.0,
          );
          // Check the padding between the suffix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            tester.getRect(findSuffixIconInnerRichText()).left - getInputRect(tester).right,
            16.0,
          );
        });

        testWidgets('suffixIcon is correctly positioned - RTL', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              textDirection: TextDirection.rtl,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          // By default, the suffix icon is rendered inside a 48x48 constrained box.
          expect(getSuffixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getSuffixIconStyle(tester).fontSize, 24.0);
          // Suffix icon is vertically centered inside the container.
          expect(getSuffixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Left padding is 12 per Material 3 spec.
          expect(tester.getRect(findSuffixIconInnerRichText()).left, 12.0);
          // Check the padding between the suffix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            getInputRect(tester).left - tester.getRect(findSuffixIconInnerRichText()).right,
            16.0,
          );
        });

        testWidgets('suffixIcon has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findSuffixIcon()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          expect(getSuffixIconStyle(tester).color, expectedColor);
        });
      });

      group('when field is disabled', () {
        testWidgets('prefixIcon is correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                enabled: false,
                prefixIcon: Icon(prefixIcon),
                labelText: labelText,
              ),
            ),
          );

          // By default, the prefix icon is rendered inside a 48x48 constrained box.
          expect(getPrefixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getPrefixIconStyle(tester).fontSize, 24.0);
          // Prefix icon is vertically centered inside the container.
          expect(getPrefixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Left padding is 12 per Material 3 spec.
          expect(tester.getRect(findPrefixIconInnerRichText()).left, 12.0);
          // Check the padding between the prefix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            getInputRect(tester).left - tester.getRect(findPrefixIconInnerRichText()).right,
            16.0,
          );
        });

        testWidgets('prefixIcon has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                enabled: false,
                labelText: labelText,
                prefixIcon: Icon(prefixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findPrefixIcon()));
          final Color expectedColor = theme.colorScheme.onSurface.withOpacity(0.38);
          expect(getPrefixIconStyle(tester).color, expectedColor);
        });

        testWidgets('suffixIcon is correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                enabled: false,
                labelText: labelText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          // By default, the suffix icon is rendered inside a 48x48 constrained box.
          expect(getSuffixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getSuffixIconStyle(tester).fontSize, 24.0);
          // Suffix icon is vertically centered inside the container.
          expect(getSuffixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Right padding is 12 per Material 3 spec.
          expect(
            getDecoratorRect(tester).right - tester.getRect(findSuffixIconInnerRichText()).right,
            12.0,
          );
          // Check the padding between the suffix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            tester.getRect(findSuffixIconInnerRichText()).left - getInputRect(tester).right,
            16.0,
          );
        });

        testWidgets('suffixIcon has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                enabled: false,
                labelText: labelText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findSuffixIcon()));
          final Color expectedColor = theme.colorScheme.onSurface.withOpacity(0.38);
          expect(getSuffixIconStyle(tester).color, expectedColor);
        });
      });

      group('when field is hovered', () {
        testWidgets('prefixIcon is correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                prefixIcon: Icon(prefixIcon),
              ),
            ),
          );

          // By default, the prefix icon is rendered inside a 48x48 constrained box.
          expect(getPrefixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getPrefixIconStyle(tester).fontSize, 24.0);
          // Prefix icon is vertically centered inside the container.
          expect(getPrefixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Left padding is 12 per Material 3 spec.
          expect(tester.getRect(findPrefixIconInnerRichText()).left, 12.0);
          // Check the padding between the prefix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            getInputRect(tester).left - tester.getRect(findPrefixIconInnerRichText()).right,
            16.0,
          );
        });

        testWidgets('prefixIcon has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                prefixIcon: Icon(prefixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findPrefixIcon()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          expect(getPrefixIconStyle(tester).color, expectedColor);
        });

        testWidgets('suffixIcon is correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          // By default, the suffix icon is rendered inside a 48x48 constrained box.
          expect(getSuffixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getSuffixIconStyle(tester).fontSize, 24.0);
          // Suffix icon is vertically centered inside the container.
          expect(getSuffixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Right padding is 12 per Material 3 spec.
          expect(
            getDecoratorRect(tester).right - tester.getRect(findSuffixIconInnerRichText()).right,
            12.0,
          );
          // Check the padding between the suffix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            tester.getRect(findSuffixIconInnerRichText()).left - getInputRect(tester).right,
            16.0,
          );
        });

        testWidgets('suffixIcon has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findSuffixIcon()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          expect(getSuffixIconStyle(tester).color, expectedColor);
        });
      });

      group('when field is focused', () {
        testWidgets('prefixIcon is correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                prefixIcon: Icon(prefixIcon),
              ),
            ),
          );

          // By default, the prefix icon is rendered inside a 48x48 constrained box.
          expect(getPrefixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getPrefixIconStyle(tester).fontSize, 24.0);
          // Prefix icon is vertically centered inside the container.
          expect(getPrefixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Left padding is 12 per Material 3 spec.
          expect(tester.getRect(findPrefixIconInnerRichText()).left, 12.0);
          // Check the padding between the prefix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            getInputRect(tester).left - tester.getRect(findPrefixIconInnerRichText()).right,
            16.0,
          );
        });

        testWidgets('prefixIcon has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                prefixIcon: Icon(prefixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findPrefixIcon()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          expect(getPrefixIconStyle(tester).color, expectedColor);
        });

        testWidgets('suffixIcon is correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          // By default, the suffix icon is rendered inside a 48x48 constrained box.
          expect(getSuffixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getSuffixIconStyle(tester).fontSize, 24.0);
          // Suffix icon is vertically centered inside the container.
          expect(getSuffixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Right padding is 12 per Material 3 spec.
          expect(
            getDecoratorRect(tester).right - tester.getRect(findSuffixIconInnerRichText()).right,
            12.0,
          );
          // Check the padding between the suffix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            tester.getRect(findSuffixIconInnerRichText()).left - getInputRect(tester).right,
            16.0,
          );
        });

        testWidgets('suffixIcon has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findSuffixIcon()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          expect(getSuffixIconStyle(tester).color, expectedColor);
        });
      });

      group('when field is in error', () {
        testWidgets('prefixIcon is correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                errorText: errorText,
                prefixIcon: Icon(prefixIcon),
              ),
            ),
          );

          // By default, the prefix icon is rendered inside a 48x48 constrained box.
          expect(getPrefixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getPrefixIconStyle(tester).fontSize, 24.0);
          // Suffix icon is vertically centered inside the container.
          expect(getPrefixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Left padding is 12 per Material 3 spec.
          expect(tester.getRect(findPrefixIconInnerRichText()).left, 12.0);
          // Check the padding between the prefix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            getInputRect(tester).left - tester.getRect(findPrefixIconInnerRichText()).right,
            16.0,
          );
        });

        testWidgets('prefixIcon has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                errorText: errorText,
                prefixIcon: Icon(prefixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findPrefixIcon()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          expect(getPrefixIconStyle(tester).color, expectedColor);
        });

        testWidgets('prefixIcon has correct color when hovered', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                errorText: errorText,
                prefixIcon: Icon(prefixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findPrefixIcon()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          expect(getPrefixIconStyle(tester).color, expectedColor);
        });

        testWidgets('suffixIcon is correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                errorText: errorText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          // By default, the suffix icon is rendered inside a 48x48 constrained box.
          expect(getSuffixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getSuffixIconStyle(tester).fontSize, 24.0);
          // Suffix icon is vertically centered inside the container.
          expect(getSuffixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Right padding is 12 per Material 3 spec.
          expect(
            getDecoratorRect(tester).right - tester.getRect(findSuffixIconInnerRichText()).right,
            12.0,
          );
          // Check the padding between the suffix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            tester.getRect(findSuffixIconInnerRichText()).left - getInputRect(tester).right,
            16.0,
          );
        });

        testWidgets('suffixIcon has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                errorText: errorText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findSuffixIcon()));
          final Color expectedColor = theme.colorScheme.error;
          expect(getSuffixIconStyle(tester).color, expectedColor);
        });

        testWidgets('suffixIcon has correct color when hovered', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                filled: true,
                labelText: labelText,
                errorText: errorText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findSuffixIcon()));
          final Color expectedColor = theme.colorScheme.onErrorContainer;
          expect(getSuffixIconStyle(tester).color, expectedColor);
        });
      });
    });

    group('for outlined text field', () {
      group('when field is enabled', () {
        testWidgets('prefixIcon is correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                prefixIcon: Icon(prefixIcon),
              ),
            ),
          );

          // By default, the prefix icon is rendered inside a 48x48 constrained box.
          expect(getPrefixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getPrefixIconStyle(tester).fontSize, 24.0);
          // Prefix icon is vertically centered inside the container.
          expect(getPrefixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Left padding is 12 per Material 3 spec.
          expect(tester.getRect(findPrefixIconInnerRichText()).left, 12.0);
          // Check the padding between the prefix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            getInputRect(tester).left - tester.getRect(findPrefixIconInnerRichText()).right,
            16.0,
          );
        });

        testWidgets('prefixIcon has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                prefixIcon: Icon(prefixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findPrefixIcon()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          expect(getPrefixIconStyle(tester).color, expectedColor);
        });

        testWidgets('suffixIcon is correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          // By default, the suffix icon is rendered inside a 48x48 constrained box.
          expect(getSuffixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getSuffixIconStyle(tester).fontSize, 24.0);
          // Suffix icon is vertically centered inside the container.
          expect(getSuffixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Right padding is 12 per Material 3 spec.
          expect(
            getDecoratorRect(tester).right - tester.getRect(findSuffixIconInnerRichText()).right,
            12.0,
          );
          // Check the padding between the suffix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            tester.getRect(findSuffixIconInnerRichText()).left - getInputRect(tester).right,
            16.0,
          );
        });

        testWidgets('suffixIcon has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findSuffixIcon()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          expect(getSuffixIconStyle(tester).color, expectedColor);
        });
      });

      group('when field is disabled', () {
        testWidgets('prefixIcon is correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                enabled: false,
                prefixIcon: Icon(prefixIcon),
                labelText: labelText,
              ),
            ),
          );

          // By default, the prefix icon is rendered inside a 48x48 constrained box.
          expect(getPrefixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getPrefixIconStyle(tester).fontSize, 24.0);
          // Prefix icon is vertically centered inside the container.
          expect(getPrefixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Left padding is 12 per Material 3 spec.
          expect(tester.getRect(findPrefixIconInnerRichText()).left, 12.0);
          // Check the padding between the prefix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            getInputRect(tester).left - tester.getRect(findPrefixIconInnerRichText()).right,
            16.0,
          );
        });

        testWidgets('prefixIcon has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                enabled: false,
                labelText: labelText,
                prefixIcon: Icon(prefixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findPrefixIcon()));
          final Color expectedColor = theme.colorScheme.onSurface.withOpacity(0.38);
          expect(getPrefixIconStyle(tester).color, expectedColor);
        });

        testWidgets('suffixIcon is correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                enabled: false,
                labelText: labelText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          // By default, the suffix icon is rendered inside a 48x48 constrained box.
          expect(getSuffixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getSuffixIconStyle(tester).fontSize, 24.0);
          // Suffix icon is vertically centered inside the container.
          expect(getSuffixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Right padding is 12 per Material 3 spec.
          expect(
            getDecoratorRect(tester).right - tester.getRect(findSuffixIconInnerRichText()).right,
            12.0,
          );
          // Check the padding between the suffix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            tester.getRect(findSuffixIconInnerRichText()).left - getInputRect(tester).right,
            16.0,
          );
        });

        testWidgets('suffixIcon has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                enabled: false,
                labelText: labelText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findSuffixIcon()));
          final Color expectedColor = theme.colorScheme.onSurface.withOpacity(0.38);
          expect(getSuffixIconStyle(tester).color, expectedColor);
        });
      });

      group('when field is hovered', () {
        testWidgets('prefixIcon is correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                prefixIcon: Icon(prefixIcon),
              ),
            ),
          );

          // By default, the prefix icon is rendered inside a 48x48 constrained box.
          expect(getPrefixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getPrefixIconStyle(tester).fontSize, 24.0);
          // Prefix icon is vertically centered inside the container.
          expect(getPrefixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Left padding is 12 per Material 3 spec.
          expect(tester.getRect(findPrefixIconInnerRichText()).left, 12.0);
          // Check the padding between the prefix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            getInputRect(tester).left - tester.getRect(findPrefixIconInnerRichText()).right,
            16.0,
          );
        });

        testWidgets('prefixIcon has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                prefixIcon: Icon(prefixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findPrefixIcon()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          expect(getPrefixIconStyle(tester).color, expectedColor);
        });

        testWidgets('suffixIcon is correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          // By default, the suffix icon is rendered inside a 48x48 constrained box.
          expect(getSuffixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getSuffixIconStyle(tester).fontSize, 24.0);
          // Suffix icon is vertically centered inside the container.
          expect(getSuffixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Right padding is 12 per Material 3 spec.
          expect(
            getDecoratorRect(tester).right - tester.getRect(findSuffixIconInnerRichText()).right,
            12.0,
          );
          // Check the padding between the suffix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            tester.getRect(findSuffixIconInnerRichText()).left - getInputRect(tester).right,
            16.0,
          );
        });

        testWidgets('suffixIcon has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findSuffixIcon()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          expect(getSuffixIconStyle(tester).color, expectedColor);
        });
      });

      group('when field is focused', () {
        testWidgets('prefixIcon is correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                prefixIcon: Icon(prefixIcon),
              ),
            ),
          );

          // By default, the prefix icon is rendered inside a 48x48 constrained box.
          expect(getPrefixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getPrefixIconStyle(tester).fontSize, 24.0);
          // Prefix icon is vertically centered inside the container.
          expect(getPrefixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Left padding is 12 per Material 3 spec.
          expect(tester.getRect(findPrefixIconInnerRichText()).left, 12.0);
          // Check the padding between the prefix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            getInputRect(tester).left - tester.getRect(findPrefixIconInnerRichText()).right,
            16.0,
          );
        });

        testWidgets('prefixIcon has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                prefixIcon: Icon(prefixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findPrefixIcon()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          expect(getPrefixIconStyle(tester).color, expectedColor);
        });

        testWidgets('suffixIcon is correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          // By default, the suffix icon is rendered inside a 48x48 constrained box.
          expect(getSuffixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getSuffixIconStyle(tester).fontSize, 24.0);
          // Suffix icon is vertically centered inside the container.
          expect(getSuffixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Right padding is 12 per Material 3 spec.
          expect(
            getDecoratorRect(tester).right - tester.getRect(findSuffixIconInnerRichText()).right,
            12.0,
          );
          // Check the padding between the suffix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            tester.getRect(findSuffixIconInnerRichText()).left - getInputRect(tester).right,
            16.0,
          );
        });

        testWidgets('suffixIcon has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isFocused: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findSuffixIcon()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          expect(getSuffixIconStyle(tester).color, expectedColor);
        });
      });

      group('when field is in error', () {
        testWidgets('prefixIcon is correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                errorText: errorText,
                prefixIcon: Icon(prefixIcon),
              ),
            ),
          );

          // By default, the prefix icon is rendered inside a 48x48 constrained box.
          expect(getPrefixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getPrefixIconStyle(tester).fontSize, 24.0);
          // Suffix icon is vertically centered inside the container.
          expect(getPrefixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Left padding is 12 per Material 3 spec.
          expect(tester.getRect(findPrefixIconInnerRichText()).left, 12.0);
          // Check the padding between the prefix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            getInputRect(tester).left - tester.getRect(findPrefixIconInnerRichText()).right,
            16.0,
          );
        });

        testWidgets('prefixIcon has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                errorText: errorText,
                prefixIcon: Icon(prefixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findPrefixIcon()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          expect(getPrefixIconStyle(tester).color, expectedColor);
        });

        testWidgets('prefixIcon has correct color when hovered', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                errorText: errorText,
                prefixIcon: Icon(prefixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findPrefixIcon()));
          final Color expectedColor = theme.colorScheme.onSurfaceVariant;
          expect(getPrefixIconStyle(tester).color, expectedColor);
        });

        testWidgets('suffixIcon is correctly positioned', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                errorText: errorText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          // By default, the suffix icon is rendered inside a 48x48 constrained box.
          expect(getSuffixIconRect(tester).size, const Size(48.0, 48.0));
          // The icon size is 24 per Material 3 spec.
          expect(getSuffixIconStyle(tester).fontSize, 24.0);
          // Suffix icon is vertically centered inside the container.
          expect(getSuffixIconRect(tester).center.dy, getContainerRect(tester).center.dy);
          // Right padding is 12 per Material 3 spec.
          expect(
            getDecoratorRect(tester).right - tester.getRect(findSuffixIconInnerRichText()).right,
            12.0,
          );
          // Check the padding between the suffix icon and the input.
          // The gap between the icon and the input should be 16 based on M3 specification.
          expect(
            tester.getRect(findSuffixIconInnerRichText()).left - getInputRect(tester).right,
            16.0,
          );
        });

        testWidgets('suffixIcon has correct color', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                errorText: errorText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findSuffixIcon()));
          final Color expectedColor = theme.colorScheme.error;
          expect(getSuffixIconStyle(tester).color, expectedColor);
        });

        testWidgets('suffixIcon has correct color when hovered', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecorator(
              isHovering: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                labelText: labelText,
                errorText: errorText,
                suffixIcon: Icon(suffixIcon),
              ),
            ),
          );

          final ThemeData theme = Theme.of(tester.element(findSuffixIcon()));
          final Color expectedColor = theme.colorScheme.onErrorContainer;
          expect(getSuffixIconStyle(tester).color, expectedColor);
        });
      });
    });

    testWidgets('InputDecorator iconColor/prefixIconColor/suffixIconColor', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: TextField(
              decoration: InputDecoration(
                icon: Icon(Icons.cabin),
                prefixIcon: Icon(Icons.sailing),
                suffixIcon: Icon(Icons.close),
                iconColor: Colors.amber,
                prefixIconColor: Colors.green,
                suffixIconColor: Colors.red,
                filled: true,
              ),
            ),
          ),
        ),
      );

      expect(
        tester.widget<IconTheme>(find.widgetWithIcon(IconTheme, Icons.cabin).first).data.color,
        Colors.amber,
      );
      expect(
        tester.widget<IconTheme>(find.widgetWithIcon(IconTheme, Icons.sailing).first).data.color,
        Colors.green,
      );
      expect(
        tester.widget<IconTheme>(find.widgetWithIcon(IconTheme, Icons.close).first).data.color,
        Colors.red,
      );
    });

    // This is a regression test for https://github.com/flutter/flutter/issues/139916.
    testWidgets('Prefix ignores pointer when hidden', (WidgetTester tester) async {
      bool tapped = false;

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return TextField(
                  decoration: InputDecoration(
                    labelText: 'label',
                    prefix: GestureDetector(
                      onTap: () {
                        setState(() {
                          tapped = true;
                        });
                      },
                      child: const Icon(Icons.search),
                    ),
                  ),
                );
              },
            ),
          ),
        ),
      );

      expect(tapped, isFalse);

      double prefixOpacity =
          tester
              .widget<AnimatedOpacity>(
                find.ancestor(of: find.byType(Icon), matching: find.byType(AnimatedOpacity)),
              )
              .opacity;

      // Initially the prefix icon should be hidden.
      expect(prefixOpacity, 0.0);

      await tester.tap(find.byType(Icon), warnIfMissed: false); // Not expected to find the target.
      await tester.pump();

      // The suffix icon should ignore pointer events when hidden.
      expect(tapped, isFalse);

      // Tap the text field to show the prefix icon.
      await tester.tap(find.byType(TextField));
      await tester.pump();

      prefixOpacity =
          tester
              .widget<AnimatedOpacity>(
                find.ancestor(of: find.byType(Icon), matching: find.byType(AnimatedOpacity)),
              )
              .opacity;

      // The prefix icon should be visible.
      expect(prefixOpacity, 1.0);

      // Tap the prefix icon.
      await tester.tap(find.byType(Icon));
      await tester.pump();

      // The prefix icon should be tapped.
      expect(tapped, isTrue);
    });

    // This is a regression test for https://github.com/flutter/flutter/issues/139916.
    testWidgets('Suffix ignores pointer when hidden', (WidgetTester tester) async {
      bool tapped = false;

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return TextField(
                  decoration: InputDecoration(
                    labelText: 'label',
                    suffix: GestureDetector(
                      onTap: () {
                        setState(() {
                          tapped = true;
                        });
                      },
                      child: const Icon(Icons.search),
                    ),
                  ),
                );
              },
            ),
          ),
        ),
      );

      expect(tapped, isFalse);

      double suffixOpacity =
          tester
              .widget<AnimatedOpacity>(
                find.ancestor(of: find.byType(Icon), matching: find.byType(AnimatedOpacity)),
              )
              .opacity;

      // Initially the suffix icon should be hidden.
      expect(suffixOpacity, 0.0);

      await tester.tap(find.byType(Icon), warnIfMissed: false); // Not expected to find the target.
      await tester.pump();

      // The suffix icon should ignore pointer events when hidden.
      expect(tapped, isFalse);

      // Tap the text field to show the suffix icon.
      await tester.tap(find.byType(TextField));
      await tester.pump();

      suffixOpacity =
          tester
              .widget<AnimatedOpacity>(
                find.ancestor(of: find.byType(Icon), matching: find.byType(AnimatedOpacity)),
              )
              .opacity;

      // The suffix icon should be visible.
      expect(suffixOpacity, 1.0);

      // Tap the suffix icon.
      await tester.tap(find.byType(Icon));
      await tester.pump();

      // The suffix icon should be tapped.
      expect(tapped, isTrue);
    });
  });

  group('Material3 - InputDecoration collapsed', () {
    // Overall height for a collapsed InputDecorator is 24dp which is the input
    // height (font size = 16, line height = 1.5).
    const double inputHeight = 24.0;

    testWidgets('Decoration height is set to input height on mobile', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecorator(decoration: const InputDecoration.collapsed(hintText: hintText)),
      );

      expect(getDecoratorRect(tester).size, const Size(800.0, inputHeight));
      expect(getInputRect(tester).height, inputHeight);
      expect(getInputRect(tester).top, 0.0);
      expect(getHintOpacity(tester), 0.0);

      // The hint should appear.
      await tester.pumpWidget(
        buildInputDecorator(
          isEmpty: true,
          isFocused: true,
          decoration: const InputDecoration.collapsed(hintText: hintText),
        ),
      );
      await tester.pumpAndSettle();

      expect(getDecoratorRect(tester).size, const Size(800.0, inputHeight));
      expect(getInputRect(tester).height, inputHeight);
      expect(getInputRect(tester).top, 0.0);
      expect(getHintOpacity(tester), 1.0);
      expect(getHintRect(tester).height, inputHeight);
      expect(getHintRect(tester).top, 0.0);
    }, variant: TargetPlatformVariant.mobile());

    testWidgets('Decoration height is set to input height on desktop', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/150763.
      await tester.pumpWidget(
        buildInputDecorator(decoration: const InputDecoration.collapsed(hintText: hintText)),
      );

      expect(getDecoratorRect(tester).size, const Size(800.0, inputHeight));
      expect(getInputRect(tester).height, inputHeight);
      expect(getInputRect(tester).top, 0.0);
      expect(getHintOpacity(tester), 0.0);

      // The hint should appear.
      await tester.pumpWidget(
        buildInputDecorator(
          isEmpty: true,
          isFocused: true,
          decoration: const InputDecoration.collapsed(hintText: hintText),
        ),
      );
      await tester.pumpAndSettle();

      expect(getDecoratorRect(tester).size, const Size(800.0, inputHeight));
      expect(getInputRect(tester).height, inputHeight);
      expect(getInputRect(tester).top, 0.0);
      expect(getHintOpacity(tester), 1.0);
      expect(getHintRect(tester).height, inputHeight);
      expect(getHintRect(tester).top, 0.0);
    }, variant: TargetPlatformVariant.desktop());

    testWidgets('InputDecoration.collapsed defaults to no border', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecorator(decoration: const InputDecoration.collapsed(hintText: hintText)),
      );

      expect(getBorderWeight(tester), 0.0);
    });

    testWidgets('InputDecoration.collapsed accepts constraints', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecorator(
          decoration: const InputDecoration.collapsed(
            hintText: hintText,
            constraints: BoxConstraints.tightFor(width: 200.0, height: 32.0),
          ),
        ),
      );

      expect(getDecoratorRect(tester).size, const Size(200.0, 32.0));
    });

    testWidgets('InputDecoration.collapsed accepts hintMaxLines', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecorator(
          decoration: const InputDecoration.collapsed(hintText: threeLines, hintMaxLines: 2),
        ),
      );

      const double hintLineHeight = 24.0; // font size = 16 and font height = 1.5.
      expect(getDecoratorRect(tester).size, const Size(800.0, 2 * hintLineHeight));
    });

    testWidgets('InputDecoration.collapsed accepts hintFadeDuration', (WidgetTester tester) async {
      // Build once with empty content.
      await tester.pumpWidget(
        buildInputDecorator(
          isEmpty: true,
          decoration: const InputDecoration.collapsed(
            hintText: hintText,
            hintFadeDuration: Duration(milliseconds: 120),
          ),
        ),
      );

      // Hint is visible (opacity 1.0).
      expect(getHintOpacity(tester), 1.0);

      // Rebuild with non-empty content.
      await tester.pumpWidget(
        buildInputDecorator(
          decoration: const InputDecoration.collapsed(
            hintText: hintText,
            hintFadeDuration: Duration(milliseconds: 120),
          ),
        ),
      );

      // The hint's opacity animates from 1.0 to 0.0.
      // The animation's default duration is 20ms.
      await tester.pump(const Duration(milliseconds: 50));
      final double hintOpacity50ms = getHintOpacity(tester);
      expect(hintOpacity50ms, inExclusiveRange(0.0, 1.0));
      await tester.pump(const Duration(milliseconds: 50));
      final double hintOpacity100ms = getHintOpacity(tester);
      expect(hintOpacity100ms, inExclusiveRange(0.0, hintOpacity50ms));
      await tester.pump(const Duration(milliseconds: 50));
      expect(getHintOpacity(tester), 0.0);
    });

    test('InputDecorationTheme.isCollapsed is applied', () {
      final InputDecoration decoration = const InputDecoration(
        hintText: 'Hello, Flutter!',
      ).applyDefaults(const InputDecorationTheme(isCollapsed: true));

      expect(decoration.isCollapsed, true);
    });

    test('InputDecorationTheme.isCollapsed defaults to false', () {
      final InputDecoration decoration = const InputDecoration(
        hintText: 'Hello, Flutter!',
      ).applyDefaults(const InputDecorationTheme());

      expect(decoration.isCollapsed, false);
    });

    test('InputDecorationTheme.isCollapsed can be overridden', () {
      final InputDecoration decoration = const InputDecoration(
        isCollapsed: true,
        hintText: 'Hello, Flutter!',
      ).applyDefaults(const InputDecorationTheme());

      expect(decoration.isCollapsed, true);
    });
  });

  testWidgets('InputDecorator counter text, widget, and null', (WidgetTester tester) async {
    Widget buildFrame({
      InputCounterWidgetBuilder? buildCounter,
      String? counterText,
      Widget? counter,
      int? maxLength,
    }) {
      return MaterialApp(
        home: Scaffold(
          body: Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[
                TextFormField(
                  buildCounter: buildCounter,
                  maxLength: maxLength,
                  decoration: InputDecoration(counterText: counterText, counter: counter),
                ),
              ],
            ),
          ),
        ),
      );
    }

    // When counter, counterText, and buildCounter are null, defaults to showing
    // the built-in counter.
    int? maxLength = 10;
    await tester.pumpWidget(buildFrame(maxLength: maxLength));
    Finder counterFinder = find.byType(Text);
    expect(counterFinder, findsOneWidget);
    final Text counterWidget = tester.widget(counterFinder);
    expect(counterWidget.data, '0/$maxLength');

    // When counter, counterText, and buildCounter are set, shows the counter
    // widget.
    final Key counterKey = UniqueKey();
    final Key buildCounterKey = UniqueKey();
    const String counterText = 'I show instead of count';
    final Widget counter = Text('hello', key: counterKey);
    Widget buildCounter(
      BuildContext context, {
      required int currentLength,
      required int? maxLength,
      required bool isFocused,
    }) {
      return Text('$currentLength of $maxLength', key: buildCounterKey);
    }

    await tester.pumpWidget(
      buildFrame(
        counterText: counterText,
        counter: counter,
        buildCounter: buildCounter,
        maxLength: maxLength,
      ),
    );
    counterFinder = find.byKey(counterKey);
    expect(counterFinder, findsOneWidget);
    expect(find.text(counterText), findsNothing);
    expect(find.byKey(buildCounterKey), findsNothing);

    // When counter is null but counterText and buildCounter are set, shows the
    // counterText.
    await tester.pumpWidget(
      buildFrame(counterText: counterText, buildCounter: buildCounter, maxLength: maxLength),
    );
    expect(find.text(counterText), findsOneWidget);
    counterFinder = find.byKey(counterKey);
    expect(counterFinder, findsNothing);
    expect(find.byKey(buildCounterKey), findsNothing);

    // When counter and counterText are null but buildCounter is set, shows the
    // generated widget.
    await tester.pumpWidget(buildFrame(buildCounter: buildCounter, maxLength: maxLength));
    expect(find.byKey(buildCounterKey), findsOneWidget);
    expect(counterFinder, findsNothing);
    expect(find.text(counterText), findsNothing);

    // When counterText is empty string and counter and buildCounter are null,
    // shows nothing.
    await tester.pumpWidget(buildFrame(counterText: '', maxLength: maxLength));
    expect(find.byType(Text), findsNothing);

    // When no maxLength, can still show a counter
    maxLength = null;
    await tester.pumpWidget(buildFrame(buildCounter: buildCounter, maxLength: maxLength));
    expect(find.byKey(buildCounterKey), findsOneWidget);
  });

  testWidgets('FloatingLabelAlignment.toString()', (WidgetTester tester) async {
    expect(FloatingLabelAlignment.start.toString(), 'FloatingLabelAlignment.start');
    expect(FloatingLabelAlignment.center.toString(), 'FloatingLabelAlignment.center');
  });

  testWidgets('InputDecorator.toString()', (WidgetTester tester) async {
    const Widget child = InputDecorator(
      key: Key('key'),
      decoration: InputDecoration(),
      baseStyle: TextStyle(),
      textAlign: TextAlign.center,
      child: Placeholder(),
    );
    expect(
      child.toString(),
      "InputDecorator-[<'key'>](decoration: InputDecoration(), baseStyle: TextStyle(<all styles inherited>), isFocused: false, isEmpty: false)",
    );
  });

  testWidgets('InputDecorator.debugDescribeChildren', (WidgetTester tester) async {
    await tester.pumpWidget(
      buildInputDecorator(
        decoration: const InputDecoration(
          icon: Text('icon'),
          labelText: 'label',
          hintText: 'hint',
          prefixText: 'prefix',
          suffixText: 'suffix',
          prefixIcon: Text('prefixIcon'),
          suffixIcon: Text('suffixIcon'),
          helperText: 'helper',
          counterText: 'counter',
        ),
        child: const Text('text'),
      ),
    );

    final RenderObject renderer = tester.renderObject(find.byType(InputDecorator));
    final Iterable<String> nodeNames = renderer.debugDescribeChildren().map(
      (DiagnosticsNode node) => node.name!,
    );
    expect(
      nodeNames,
      unorderedEquals(<String>[
        'container',
        'counter',
        'helperError',
        'hint',
        'icon',
        'input',
        'label',
        'prefix',
        'prefixIcon',
        'suffix',
        'suffixIcon',
      ]),
    );

    final Set<Object> nodeValues = Set<Object>.from(
      renderer.debugDescribeChildren().map<Object>((DiagnosticsNode node) => node.value!),
    );
    expect(nodeValues.length, 11);
  });

  testWidgets('InputDecorationTheme.applyDefaults initializes empty field', (
    WidgetTester tester,
  ) async {
    const TextStyle themeStyle = TextStyle(color: Color(0xFF00FFFF));
    const Color themeColor = Color(0xFF00FF00);
    const InputBorder themeInputBorder = OutlineInputBorder(
      borderSide: BorderSide(color: Color(0xFF0000FF)),
    );

    final InputDecoration decoration = const InputDecoration().applyDefaults(
      const InputDecorationTheme(
        labelStyle: themeStyle,
        floatingLabelStyle: themeStyle,
        helperStyle: themeStyle,
        helperMaxLines: 2,
        hintStyle: themeStyle,
        errorStyle: themeStyle,
        errorMaxLines: 2,
        floatingLabelBehavior: FloatingLabelBehavior.never,
        floatingLabelAlignment: FloatingLabelAlignment.center,
        isDense: true,
        contentPadding: EdgeInsets.all(1.0),
        iconColor: themeColor,
        prefixStyle: themeStyle,
        prefixIconColor: themeColor,
        prefixIconConstraints: BoxConstraints(
          minWidth: 10,
          maxWidth: 10,
          minHeight: 30,
          maxHeight: 30,
        ),
        suffixStyle: themeStyle,
        suffixIconColor: themeColor,
        suffixIconConstraints: BoxConstraints(
          minWidth: 20,
          maxWidth: 20,
          minHeight: 40,
          maxHeight: 40,
        ),
        counterStyle: themeStyle,
        filled: true,
        fillColor: themeColor,
        focusColor: themeColor,
        hoverColor: themeColor,
        errorBorder: themeInputBorder,
        focusedBorder: themeInputBorder,
        focusedErrorBorder: themeInputBorder,
        disabledBorder: themeInputBorder,
        enabledBorder: themeInputBorder,
        border: InputBorder.none,
        alignLabelWithHint: true,
        constraints: BoxConstraints(minWidth: 10, maxWidth: 20, minHeight: 30, maxHeight: 40),
      ),
    );

    expect(decoration.labelStyle, themeStyle);
    expect(decoration.floatingLabelStyle, themeStyle);
    expect(decoration.helperStyle, themeStyle);
    expect(decoration.helperMaxLines, 2);
    expect(decoration.hintStyle, themeStyle);
    expect(decoration.errorStyle, themeStyle);
    expect(decoration.errorMaxLines, 2);
    expect(decoration.floatingLabelBehavior, FloatingLabelBehavior.never);
    expect(decoration.floatingLabelAlignment, FloatingLabelAlignment.center);
    expect(decoration.isDense, true);
    expect(decoration.contentPadding, const EdgeInsets.all(1.0));
    expect(decoration.iconColor, themeColor);
    expect(decoration.prefixStyle, themeStyle);
    expect(decoration.prefixIconColor, themeColor);
    expect(
      decoration.prefixIconConstraints,
      const BoxConstraints(minWidth: 10, maxWidth: 10, minHeight: 30, maxHeight: 30),
    );
    expect(decoration.suffixStyle, themeStyle);
    expect(decoration.suffixIconColor, themeColor);
    expect(
      decoration.suffixIconConstraints,
      const BoxConstraints(minWidth: 20, maxWidth: 20, minHeight: 40, maxHeight: 40),
    );
    expect(decoration.counterStyle, themeStyle);
    expect(decoration.filled, true);
    expect(decoration.fillColor, themeColor);
    expect(decoration.focusColor, themeColor);
    expect(decoration.hoverColor, themeColor);
    expect(decoration.errorBorder, themeInputBorder);
    expect(decoration.focusedBorder, themeInputBorder);
    expect(decoration.focusedErrorBorder, themeInputBorder);
    expect(decoration.disabledBorder, themeInputBorder);
    expect(decoration.enabledBorder, themeInputBorder);
    expect(decoration.border, InputBorder.none);
    expect(decoration.alignLabelWithHint, true);
    expect(
      decoration.constraints,
      const BoxConstraints(minWidth: 10, maxWidth: 20, minHeight: 30, maxHeight: 40),
    );
  });

  testWidgets('InputDecorationTheme.applyDefaults does not override non-null fields', (
    WidgetTester tester,
  ) async {
    const TextStyle themeStyle = TextStyle(color: Color(0xFF00FFFF));
    const Color themeColor = Color(0xFF00FF00);
    const InputBorder themeInputBorder = OutlineInputBorder(
      borderSide: BorderSide(color: Color(0xFF0000FF)),
    );
    const TextStyle decorationStyle = TextStyle(color: Color(0xFFFFFF00));
    const Color decorationColor = Color(0xFF0000FF);
    const InputBorder decorationInputBorder = OutlineInputBorder(
      borderSide: BorderSide(color: Color(0xFFFF00FF)),
    );
    const BoxConstraints decorationConstraints = BoxConstraints(
      minWidth: 40,
      maxWidth: 50,
      minHeight: 60,
      maxHeight: 70,
    );

    final InputDecoration decoration = const InputDecoration(
      labelStyle: decorationStyle,
      floatingLabelStyle: decorationStyle,
      helperStyle: decorationStyle,
      helperMaxLines: 3,
      hintStyle: decorationStyle,
      errorStyle: decorationStyle,
      errorMaxLines: 3,
      floatingLabelBehavior: FloatingLabelBehavior.always,
      floatingLabelAlignment: FloatingLabelAlignment.start,
      isDense: false,
      contentPadding: EdgeInsets.all(4.0),
      iconColor: decorationColor,
      prefixStyle: decorationStyle,
      prefixIconColor: decorationColor,
      prefixIconConstraints: decorationConstraints,
      suffixStyle: decorationStyle,
      suffixIconColor: decorationColor,
      suffixIconConstraints: decorationConstraints,
      counterStyle: decorationStyle,
      filled: false,
      fillColor: decorationColor,
      focusColor: decorationColor,
      hoverColor: decorationColor,
      errorBorder: decorationInputBorder,
      focusedBorder: decorationInputBorder,
      focusedErrorBorder: decorationInputBorder,
      disabledBorder: decorationInputBorder,
      enabledBorder: decorationInputBorder,
      border: OutlineInputBorder(),
      alignLabelWithHint: false,
      constraints: decorationConstraints,
    ).applyDefaults(
      const InputDecorationTheme(
        labelStyle: themeStyle,
        floatingLabelStyle: themeStyle,
        helperStyle: themeStyle,
        helperMaxLines: 2,
        hintStyle: themeStyle,
        errorStyle: themeStyle,
        errorMaxLines: 2,
        floatingLabelBehavior: FloatingLabelBehavior.never,
        floatingLabelAlignment: FloatingLabelAlignment.center,
        isDense: true,
        contentPadding: EdgeInsets.all(1.0),
        iconColor: themeColor,
        prefixStyle: themeStyle,
        prefixIconColor: themeColor,
        suffixStyle: themeStyle,
        suffixIconColor: themeColor,
        counterStyle: themeStyle,
        filled: true,
        fillColor: themeColor,
        focusColor: themeColor,
        hoverColor: themeColor,
        errorBorder: themeInputBorder,
        focusedBorder: themeInputBorder,
        focusedErrorBorder: themeInputBorder,
        disabledBorder: themeInputBorder,
        enabledBorder: themeInputBorder,
        border: InputBorder.none,
        alignLabelWithHint: true,
        constraints: BoxConstraints(minWidth: 10, maxWidth: 20, minHeight: 30, maxHeight: 40),
      ),
    );

    expect(decoration.labelStyle, decorationStyle);
    expect(decoration.floatingLabelStyle, decorationStyle);
    expect(decoration.helperStyle, decorationStyle);
    expect(decoration.helperMaxLines, 3);
    expect(decoration.hintStyle, decorationStyle);
    expect(decoration.errorStyle, decorationStyle);
    expect(decoration.errorMaxLines, 3);
    expect(decoration.floatingLabelBehavior, FloatingLabelBehavior.always);
    expect(decoration.floatingLabelAlignment, FloatingLabelAlignment.start);
    expect(decoration.isDense, false);
    expect(decoration.contentPadding, const EdgeInsets.all(4.0));
    expect(decoration.iconColor, decorationColor);
    expect(decoration.prefixStyle, decorationStyle);
    expect(decoration.prefixIconColor, decorationColor);
    expect(decoration.prefixIconConstraints, decorationConstraints);
    expect(decoration.suffixStyle, decorationStyle);
    expect(decoration.suffixIconColor, decorationColor);
    expect(decoration.suffixIconConstraints, decorationConstraints);
    expect(decoration.counterStyle, decorationStyle);
    expect(decoration.filled, false);
    expect(decoration.fillColor, decorationColor);
    expect(decoration.focusColor, decorationColor);
    expect(decoration.hoverColor, decorationColor);
    expect(decoration.errorBorder, decorationInputBorder);
    expect(decoration.focusedBorder, decorationInputBorder);
    expect(decoration.focusedErrorBorder, decorationInputBorder);
    expect(decoration.disabledBorder, decorationInputBorder);
    expect(decoration.enabledBorder, decorationInputBorder);
    expect(decoration.border, const OutlineInputBorder());
    expect(decoration.alignLabelWithHint, false);
    expect(decoration.constraints, decorationConstraints);
  });

  testWidgets('InputDecorationTheme.inputDecoration with MaterialState', (
    WidgetTester tester,
  ) async {
    final MaterialStateTextStyle themeStyle = MaterialStateTextStyle.resolveWith((
      Set<MaterialState> states,
    ) {
      return const TextStyle(color: Colors.green);
    });

    final MaterialStateTextStyle decorationStyle = MaterialStateTextStyle.resolveWith((
      Set<MaterialState> states,
    ) {
      return const TextStyle(color: Colors.blue);
    });

    // InputDecorationTheme arguments define InputDecoration properties.
    InputDecoration decoration = const InputDecoration().applyDefaults(
      InputDecorationTheme(
        labelStyle: themeStyle,
        helperStyle: themeStyle,
        hintStyle: themeStyle,
        errorStyle: themeStyle,
        isDense: true,
        contentPadding: const EdgeInsets.all(1.0),
        prefixStyle: themeStyle,
        suffixStyle: themeStyle,
        counterStyle: themeStyle,
        filled: true,
        fillColor: Colors.red,
        focusColor: Colors.blue,
        border: InputBorder.none,
        alignLabelWithHint: true,
        constraints: const BoxConstraints(minWidth: 10, maxWidth: 20, minHeight: 30, maxHeight: 40),
      ),
    );

    expect(decoration.labelStyle, themeStyle);
    expect(decoration.helperStyle, themeStyle);
    expect(decoration.hintStyle, themeStyle);
    expect(decoration.errorStyle, themeStyle);
    expect(decoration.isDense, true);
    expect(decoration.contentPadding, const EdgeInsets.all(1.0));
    expect(decoration.prefixStyle, themeStyle);
    expect(decoration.suffixStyle, themeStyle);
    expect(decoration.counterStyle, themeStyle);
    expect(decoration.filled, true);
    expect(decoration.fillColor, Colors.red);
    expect(decoration.border, InputBorder.none);
    expect(decoration.alignLabelWithHint, true);
    expect(
      decoration.constraints,
      const BoxConstraints(minWidth: 10, maxWidth: 20, minHeight: 30, maxHeight: 40),
    );

    // InputDecoration (baseDecoration) defines InputDecoration properties
    final MaterialStateOutlineInputBorder border = MaterialStateOutlineInputBorder.resolveWith((
      Set<MaterialState> states,
    ) {
      return const OutlineInputBorder();
    });
    decoration = InputDecoration(
      labelStyle: decorationStyle,
      helperStyle: decorationStyle,
      hintStyle: decorationStyle,
      errorStyle: decorationStyle,
      isDense: false,
      contentPadding: const EdgeInsets.all(4.0),
      prefixStyle: decorationStyle,
      suffixStyle: decorationStyle,
      counterStyle: decorationStyle,
      filled: false,
      fillColor: Colors.blue,
      border: border,
      alignLabelWithHint: false,
      constraints: const BoxConstraints(minWidth: 10, maxWidth: 20, minHeight: 30, maxHeight: 40),
    ).applyDefaults(
      InputDecorationTheme(
        labelStyle: themeStyle,
        helperStyle: themeStyle,
        helperMaxLines: 5,
        hintStyle: themeStyle,
        errorStyle: themeStyle,
        errorMaxLines: 4,
        isDense: true,
        contentPadding: const EdgeInsets.all(1.0),
        prefixStyle: themeStyle,
        suffixStyle: themeStyle,
        counterStyle: themeStyle,
        filled: true,
        fillColor: Colors.red,
        focusColor: Colors.blue,
        border: InputBorder.none,
        alignLabelWithHint: true,
        constraints: const BoxConstraints(minWidth: 40, maxWidth: 30, minHeight: 20, maxHeight: 10),
      ),
    );

    expect(decoration.labelStyle, decorationStyle);
    expect(decoration.helperStyle, decorationStyle);
    expect(decoration.helperMaxLines, 5);
    expect(decoration.hintStyle, decorationStyle);
    expect(decoration.errorStyle, decorationStyle);
    expect(decoration.errorMaxLines, 4);
    expect(decoration.isDense, false);
    expect(decoration.contentPadding, const EdgeInsets.all(4.0));
    expect(decoration.prefixStyle, decorationStyle);
    expect(decoration.suffixStyle, decorationStyle);
    expect(decoration.counterStyle, decorationStyle);
    expect(decoration.filled, false);
    expect(decoration.fillColor, Colors.blue);
    expect(decoration.border, isA<MaterialStateOutlineInputBorder>());
    expect(decoration.alignLabelWithHint, false);
    expect(
      decoration.constraints,
      const BoxConstraints(minWidth: 10, maxWidth: 20, minHeight: 30, maxHeight: 40),
    );
  });

  testWidgets('InputDecoration with WidgetStateInputBorder', (WidgetTester tester) async {
    const WidgetStateInputBorder outlineInputBorder =
        WidgetStateInputBorder.fromMap(<WidgetStatesConstraint, InputBorder>{
          WidgetState.focused: OutlineInputBorder(
            borderSide: BorderSide(color: Colors.blue, width: 4.0),
          ),
          WidgetState.hovered: OutlineInputBorder(
            borderSide: BorderSide(color: Colors.cyan, width: 8.0),
          ),
          WidgetState.any: OutlineInputBorder(),
        });

    RenderObject getBorder() {
      return tester.renderObject(
        find.descendant(of: find.byType(TextField), matching: find.byType(CustomPaint)),
      );
    }

    final FocusNode focusNode = FocusNode();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: TextField(
            focusNode: focusNode,
            decoration: const InputDecoration(border: outlineInputBorder),
          ),
        ),
      ),
    );
    expect(getBorder(), paints..rrect(strokeWidth: 1.0));

    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(getBorder(), paints..rrect(color: Colors.blue, strokeWidth: 4.0));

    focusNode.unfocus();
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: tester.getCenter(find.byType(TextField)));
    await tester.pumpAndSettle();
    expect(getBorder(), paints..rrect(color: Colors.cyan, strokeWidth: 8.0));

    focusNode.dispose();
  });

  testWidgets('InputDecorator constrained to 0x0', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/17710
    await tester.pumpWidget(
      Material(
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: UnconstrainedBox(
            child: ConstrainedBox(
              constraints: BoxConstraints.tight(Size.zero),
              child: const InputDecorator(
                decoration: InputDecoration(labelText: 'XP', border: OutlineInputBorder()),
              ),
            ),
          ),
        ),
      ),
    );
  });

  testWidgets('InputDecorationTheme.toString()', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/19305
    expect(
      const InputDecorationTheme(contentPadding: EdgeInsetsDirectional.only(start: 5.0)).toString(),
      contains('contentPadding: EdgeInsetsDirectional(5.0, 0.0, 0.0, 0.0)'),
    );

    // Regression test for https://github.com/flutter/flutter/issues/20374
    expect(
      const InputDecorationTheme(contentPadding: EdgeInsets.only(left: 5.0)).toString(),
      contains('contentPadding: EdgeInsets(5.0, 0.0, 0.0, 0.0)'),
    );

    // Verify that the toString() method succeeds.
    final String debugString =
        const InputDecorationTheme(
          labelStyle: TextStyle(height: 1.0),
          helperStyle: TextStyle(height: 2.0),
          helperMaxLines: 5,
          hintStyle: TextStyle(height: 3.0),
          errorStyle: TextStyle(height: 4.0),
          errorMaxLines: 5,
          isDense: true,
          contentPadding: EdgeInsets.only(right: 6.0),
          isCollapsed: true,
          prefixStyle: TextStyle(height: 7.0),
          suffixStyle: TextStyle(height: 8.0),
          counterStyle: TextStyle(height: 9.0),
          filled: true,
          fillColor: Color(0x00000010),
          focusColor: Color(0x00000020),
          errorBorder: UnderlineInputBorder(),
          focusedBorder: OutlineInputBorder(),
          focusedErrorBorder: UnderlineInputBorder(),
          disabledBorder: OutlineInputBorder(),
          enabledBorder: UnderlineInputBorder(),
          border: OutlineInputBorder(),
        ).toString();

    // Spot check
    expect(debugString, contains('labelStyle: TextStyle(inherit: true, height: 1.0x)'));
    expect(debugString, contains('isDense: true'));
    expect(debugString, contains('fillColor: ${const Color(0x00000010)}'));
    expect(debugString, contains('focusColor: ${const Color(0x00000020)}'));
    expect(debugString, contains('errorBorder: UnderlineInputBorder()'));
    expect(debugString, contains('focusedBorder: OutlineInputBorder()'));
  });

  testWidgets('InputDecorationTheme implements debugFillDescription', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const BoxConstraints constraints = BoxConstraints(
      minWidth: 10,
      maxWidth: 10,
      minHeight: 30,
      maxHeight: 30,
    );
    const InputDecorationTheme(
      labelStyle: TextStyle(),
      floatingLabelStyle: TextStyle(),
      helperStyle: TextStyle(),
      helperMaxLines: 6,
      hintStyle: TextStyle(),
      errorStyle: TextStyle(),
      errorMaxLines: 5,
      floatingLabelBehavior: FloatingLabelBehavior.never,
      floatingLabelAlignment: FloatingLabelAlignment.center,
      isDense: true,
      contentPadding: EdgeInsetsDirectional.only(start: 40.0, top: 12.0, bottom: 12.0),
      isCollapsed: true,
      iconColor: Colors.red,
      prefixIconColor: Colors.blue,
      prefixIconConstraints: constraints,
      prefixStyle: TextStyle(),
      suffixIconColor: Colors.blue,
      suffixIconConstraints: constraints,
      suffixStyle: TextStyle(),
      counterStyle: TextStyle(),
      filled: true,
      fillColor: Colors.red,
      activeIndicatorBorder: BorderSide(),
      outlineBorder: BorderSide(),
      focusColor: Colors.blue,
      hoverColor: Colors.green,
      errorBorder: UnderlineInputBorder(),
      focusedBorder: UnderlineInputBorder(),
      focusedErrorBorder: UnderlineInputBorder(),
      disabledBorder: UnderlineInputBorder(),
      enabledBorder: UnderlineInputBorder(),
      border: UnderlineInputBorder(),
      alignLabelWithHint: true,
      constraints: constraints,
    ).debugFillProperties(builder);
    final List<String> description =
        builder.properties
            .where((DiagnosticsNode n) => !n.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode n) => n.toString())
            .toList();
    expect(description, <String>[
      'labelStyle: TextStyle(<all styles inherited>)',
      'floatingLabelStyle: TextStyle(<all styles inherited>)',
      'helperStyle: TextStyle(<all styles inherited>)',
      'helperMaxLines: 6',
      'hintStyle: TextStyle(<all styles inherited>)',
      'errorStyle: TextStyle(<all styles inherited>)',
      'errorMaxLines: 5',
      'floatingLabelBehavior: FloatingLabelBehavior.never',
      'floatingLabelAlignment: FloatingLabelAlignment.center',
      'isDense: true',
      'contentPadding: EdgeInsetsDirectional(40.0, 12.0, 0.0, 12.0)',
      'isCollapsed: true',
      'iconColor: MaterialColor(primary value: ${const Color(0xfff44336)})',
      'prefixIconColor: MaterialColor(primary value: ${const Color(0xff2196f3)})',
      'prefixIconConstraints: BoxConstraints(w=10.0, h=30.0)',
      'prefixStyle: TextStyle(<all styles inherited>)',
      'suffixIconColor: MaterialColor(primary value: ${const Color(0xff2196f3)})',
      'suffixIconConstraints: BoxConstraints(w=10.0, h=30.0)',
      'suffixStyle: TextStyle(<all styles inherited>)',
      'counterStyle: TextStyle(<all styles inherited>)',
      'filled: true',
      'fillColor: MaterialColor(primary value: ${const Color(0xfff44336)})',
      'activeIndicatorBorder: BorderSide',
      'outlineBorder: BorderSide',
      'focusColor: MaterialColor(primary value: ${const Color(0xff2196f3)})',
      'hoverColor: MaterialColor(primary value: ${const Color(0xff4caf50)})',
      'errorBorder: UnderlineInputBorder()',
      'focusedBorder: UnderlineInputBorder()',
      'focusedErrorBorder: UnderlineInputBorder()',
      'disabledBorder: UnderlineInputBorder()',
      'enabledBorder: UnderlineInputBorder()',
      'border: UnderlineInputBorder()',
      'alignLabelWithHint: true',
      'constraints: BoxConstraints(w=10.0, h=30.0)',
    ]);
  });

  testWidgets("InputDecorator label width isn't affected by prefix or suffix", (
    WidgetTester tester,
  ) async {
    const String labelText = 'My Label';
    const String prefixText = 'The five boxing wizards jump quickly.';
    const String suffixText = 'Suffix';

    Widget getLabeledInputDecorator(bool showFix) {
      return MaterialApp(
        home: Material(
          child: Builder(
            builder: (BuildContext context) {
              return Theme(
                data: Theme.of(context),
                child: Align(
                  alignment: Alignment.topLeft,
                  child: TextField(
                    decoration: InputDecoration(
                      icon: const Icon(Icons.assistant),
                      prefixText: showFix ? prefixText : null,
                      suffixText: showFix ? suffixText : null,
                      suffixIcon: const Icon(Icons.threesixty),
                      labelText: labelText,
                    ),
                  ),
                ),
              );
            },
          ),
        ),
      );
    }

    // Build with no prefix or suffix.
    await tester.pumpWidget(getLabeledInputDecorator(false));

    // Get the width of the label when there is no prefix/suffix.
    expect(find.text(prefixText), findsNothing);
    expect(find.text(suffixText), findsNothing);
    final double labelWidth = tester.getSize(find.text(labelText)).width;

    // Build with a prefix and suffix.
    await tester.pumpWidget(getLabeledInputDecorator(true));

    // The prefix and suffix exist but aren't visible. They have not affected
    // the width of the label.
    expect(find.text(prefixText), findsOneWidget);
    expect(getOpacity(tester, prefixText), 0.0);
    expect(find.text(suffixText), findsOneWidget);
    expect(getOpacity(tester, suffixText), 0.0);
    expect(tester.getSize(find.text(labelText)).width, labelWidth);

    // Tap to focus.
    await tester.tap(find.byType(TextField));
    // TODO(bleroux): investigate why this pumpAndSettle is required.
    await tester.pumpAndSettle();

    // The prefix and suffix are visible, and the label is floating and still
    // hasn't had its width affected.
    expect(tester.getSize(find.text(labelText)).width, labelWidth);
    expect(getOpacity(tester, prefixText), 1.0);
  });

  testWidgets('Prefix and suffix are not visible when decorator is empty', (
    WidgetTester tester,
  ) async {
    const String prefixText = 'Prefix';
    const String suffixText = 'Suffix';

    await tester.pumpWidget(
      buildInputDecorator(
        isEmpty: true,
        decoration: const InputDecoration(
          filled: true,
          labelText: labelText,
          prefixText: prefixText,
          suffixText: suffixText,
        ),
      ),
    );

    // Prefix and suffix are hidden.
    expect(getOpacity(tester, prefixText), 0.0);
    expect(getOpacity(tester, suffixText), 0.0);
  });

  testWidgets(
    'Prefix and suffix are visible when decorator is empty and floating behavior is FloatingBehavior.always',
    (WidgetTester tester) async {
      const String prefixText = 'Prefix';
      const String suffixText = 'Suffix';

      await tester.pumpWidget(
        buildInputDecorator(
          isEmpty: true,
          decoration: const InputDecoration(
            filled: true,
            labelText: labelText,
            prefixText: prefixText,
            suffixText: suffixText,
            floatingLabelBehavior: FloatingLabelBehavior.always,
          ),
        ),
      );

      // Prefix and suffix are visible.
      expect(getOpacity(tester, prefixText), 1.0);
      expect(getOpacity(tester, suffixText), 1.0);
    },
  );

  testWidgets(
    'OutlineInputBorder and InputDecorator long labels and in Floating, the width should ignore the icon width',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/64427.
      const String labelText =
          'Flutter is Google’s UI toolkit for building beautiful, natively compiled applications for mobile, web, and desktop from a single codebase.';

      Widget getLabeledInputDecorator(FloatingLabelBehavior floatingLabelBehavior) => MaterialApp(
        home: Material(
          child: SizedBox(
            width: 300,
            child: TextField(
              decoration: InputDecoration(
                border: const OutlineInputBorder(borderSide: BorderSide(color: Colors.greenAccent)),
                suffixIcon: const Icon(Icons.arrow_drop_down),
                floatingLabelBehavior: floatingLabelBehavior,
                labelText: labelText,
              ),
            ),
          ),
        ),
      );

      await tester.pumpWidget(getLabeledInputDecorator(FloatingLabelBehavior.never));

      final double labelWidth = getLabelRect(tester).width;

      await tester.pumpWidget(getLabeledInputDecorator(FloatingLabelBehavior.always));
      await tester.pump(kTransitionDuration);

      final double floatedLabelWidth = getLabelRect(tester).width;

      expect(floatedLabelWidth, greaterThan(labelWidth));

      final Widget target = getLabeledInputDecorator(FloatingLabelBehavior.auto);
      await tester.pumpWidget(target);
      await tester.pump(kTransitionDuration);

      expect(getLabelRect(tester).width, labelWidth);

      // Click for Focus.
      await tester.tap(find.byType(TextField));
      // Default animation duration is 167ms.
      await tester.pumpFrames(target, const Duration(milliseconds: 80));

      expect(getLabelRect(tester).width, greaterThan(labelWidth));
      expect(getLabelRect(tester).width, lessThanOrEqualTo(floatedLabelWidth));

      await tester.pump(kTransitionDuration);

      expect(getLabelRect(tester).width, floatedLabelWidth);
    },
  );

  testWidgets(
    'given enough space, constrained and unconstrained heights result in the same size widget',
    (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/65572.
      final UniqueKey keyUnconstrained = UniqueKey();
      final UniqueKey keyConstrained = UniqueKey();

      Widget getInputDecorator(VisualDensity visualDensity) {
        return MaterialApp(
          home: Material(
            child: Builder(
              builder: (BuildContext context) {
                return Theme(
                  data: Theme.of(context).copyWith(visualDensity: visualDensity),
                  child: Center(
                    child: Row(
                      children: <Widget>[
                        SizedBox(width: 35.0, child: TextField(key: keyUnconstrained)),
                        SizedBox(
                          width: 35.0,
                          // 48 is the height that this TextField would take when
                          // laid out with no constraints.
                          height: 48.0,
                          child: TextField(key: keyConstrained),
                        ),
                      ],
                    ),
                  ),
                );
              },
            ),
          ),
        );
      }

      await tester.pumpWidget(getInputDecorator(VisualDensity.standard));
      final double constrainedHeight = tester.getSize(find.byKey(keyConstrained)).height;
      final double unConstrainedHeight = tester.getSize(find.byKey(keyUnconstrained)).height;
      expect(constrainedHeight, equals(unConstrainedHeight));

      await tester.pumpWidget(getInputDecorator(VisualDensity.compact));
      final double constrainedHeightCompact = tester.getSize(find.byKey(keyConstrained)).height;
      final double unConstrainedHeightCompact = tester.getSize(find.byKey(keyUnconstrained)).height;
      expect(constrainedHeightCompact, equals(unConstrainedHeightCompact));
    },
  );

  testWidgets('A vertically constrained TextField still positions its text inside of itself', (
    WidgetTester tester,
  ) async {
    final TextEditingController controller = TextEditingController(text: 'A');
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: SizedBox(width: 200, height: 28, child: TextField(controller: controller)),
          ),
        ),
      ),
    );

    final double textFieldTop = tester.getTopLeft(find.byType(TextField)).dy;
    final double textFieldBottom = tester.getBottomLeft(find.byType(TextField)).dy;
    final double textTop = tester.getTopLeft(find.text('A')).dy;

    // The text is inside the field.
    expect(tester.getSize(find.text('A')).height, lessThan(textFieldBottom - textFieldTop));
    expect(textTop, greaterThan(textFieldTop));
    expect(textTop, lessThan(textFieldBottom));
  });

  testWidgets('Visual density is included in the intrinsic height calculation', (
    WidgetTester tester,
  ) async {
    final UniqueKey key = UniqueKey();
    final UniqueKey intrinsicHeightKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Builder(
            builder: (BuildContext context) {
              return Theme(
                data: Theme.of(context).copyWith(visualDensity: VisualDensity.compact),
                child: Center(
                  child: Row(
                    children: <Widget>[
                      SizedBox(width: 35.0, child: TextField(key: key)),
                      SizedBox(
                        width: 35.0,
                        child: IntrinsicHeight(child: TextField(key: intrinsicHeightKey)),
                      ),
                    ],
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    final double height = tester.getSize(find.byKey(key)).height;
    final double intrinsicHeight = tester.getSize(find.byKey(intrinsicHeightKey)).height;
    expect(intrinsicHeight, equals(height));
  });

  testWidgets('Min intrinsic height for TextField with no content padding', (
    WidgetTester tester,
  ) async {
    // Regression test for: https://github.com/flutter/flutter/issues/75509
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: Center(
            child: IntrinsicHeight(
              child: Column(
                children: <Widget>[
                  TextField(
                    decoration: InputDecoration(
                      labelText: 'Label Text',
                      helperText: 'Helper Text',
                      contentPadding: EdgeInsets.zero,
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      ),
    );

    expect(tester.takeException(), isNull);
  });

  testWidgets('Min intrinsic height for TextField with prefix icon', (WidgetTester tester) async {
    final TextEditingController controller = TextEditingController(text: 'input');
    addTearDown(controller.dispose);

    // Regression test for: https://github.com/flutter/flutter/issues/87403
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: SizedBox(
              width: 100.0,
              child: IntrinsicHeight(
                child: Column(
                  children: <Widget>[
                    TextField(
                      controller: controller,
                      maxLines: null,
                      decoration: const InputDecoration(prefixIcon: Icon(Icons.search)),
                    ),
                  ],
                ),
              ),
            ),
          ),
        ),
      ),
    );

    expect(tester.takeException(), isNull);
  });

  testWidgets('Min intrinsic height for TextField with suffix icon', (WidgetTester tester) async {
    final TextEditingController controller = TextEditingController(text: 'input');
    addTearDown(controller.dispose);

    // Regression test for: https://github.com/flutter/flutter/issues/87403
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: SizedBox(
              width: 100.0,
              child: IntrinsicHeight(
                child: Column(
                  children: <Widget>[
                    TextField(
                      controller: controller,
                      maxLines: null,
                      decoration: const InputDecoration(suffixIcon: Icon(Icons.search)),
                    ),
                  ],
                ),
              ),
            ),
          ),
        ),
      ),
    );

    expect(tester.takeException(), isNull);
  });

  testWidgets('Min intrinsic height for TextField with prefix', (WidgetTester tester) async {
    final TextEditingController controller = TextEditingController(text: 'input');
    addTearDown(controller.dispose);

    // Regression test for: https://github.com/flutter/flutter/issues/87403
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: SizedBox(
              width: 100.0,
              child: IntrinsicHeight(
                child: Column(
                  children: <Widget>[
                    TextField(
                      controller: controller,
                      maxLines: null,
                      decoration: const InputDecoration(prefix: Text('prefix')),
                    ),
                  ],
                ),
              ),
            ),
          ),
        ),
      ),
    );

    expect(tester.takeException(), isNull);
  });

  testWidgets('Min intrinsic height for TextField with suffix', (WidgetTester tester) async {
    final TextEditingController controller = TextEditingController(text: 'input');
    addTearDown(controller.dispose);

    // Regression test for: https://github.com/flutter/flutter/issues/87403
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: SizedBox(
              width: 100.0,
              child: IntrinsicHeight(
                child: Column(
                  children: <Widget>[
                    TextField(
                      controller: controller,
                      maxLines: null,
                      decoration: const InputDecoration(suffix: Text('suffix')),
                    ),
                  ],
                ),
              ),
            ),
          ),
        ),
      ),
    );

    expect(tester.takeException(), isNull);
  });

  testWidgets('Min intrinsic height for TextField with icon', (WidgetTester tester) async {
    final TextEditingController controller = TextEditingController(text: 'input');
    addTearDown(controller.dispose);

    // Regression test for: https://github.com/flutter/flutter/issues/87403
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: SizedBox(
              width: 100.0,
              child: IntrinsicHeight(
                child: Column(
                  children: <Widget>[
                    TextField(
                      controller: controller,
                      maxLines: null,
                      decoration: const InputDecoration(icon: Icon(Icons.search)),
                    ),
                  ],
                ),
              ),
            ),
          ),
        ),
      ),
    );

    expect(tester.takeException(), isNull);
  });

  group('Intrinsic width', () {
    const EdgeInsetsGeometry padding = EdgeInsetsDirectional.only(end: 24, start: 12);

    const InputDecoration decorationWithoutIcons = InputDecoration(contentPadding: padding);
    const InputDecoration decorationWithPrefix = InputDecoration(
      contentPadding: padding,
      prefixIcon: Icon(Icons.search),
    );
    const InputDecoration decorationWithSuffix = InputDecoration(
      contentPadding: padding,
      suffixIcon: Icon(Icons.search),
    );
    const InputDecoration decorationWithAffixes = InputDecoration(
      contentPadding: padding,
      prefixIcon: Icon(Icons.search),
      suffixIcon: Icon(Icons.search),
    );

    Future<Size> measureText(
      WidgetTester tester,
      InputDecoration decoration,
      TextDirection direction,
    ) async {
      await tester.pumpWidget(
        buildInputDecorator(
          decoration: decoration,
          useIntrinsicWidth: true,
          textDirection: direction,
        ),
      );
      return tester.renderObject<RenderBox>(findInputText()).size;
    }

    testWidgets('with prefixIcon in LTR', (WidgetTester tester) async {
      final Size textSizeWithoutIcon = await measureText(
        tester,
        decorationWithoutIcons,
        TextDirection.ltr,
      );
      final Size textSizeWithPrefixIcon = await measureText(
        tester,
        decorationWithPrefix,
        TextDirection.ltr,
      );

      expect(textSizeWithPrefixIcon.width, equals(textSizeWithoutIcon.width));
    });

    testWidgets('with suffixIcon in LTR', (WidgetTester tester) async {
      final Size textSizeWithoutIcon = await measureText(
        tester,
        decorationWithoutIcons,
        TextDirection.ltr,
      );
      final Size textSizeWithSuffixIcon = await measureText(
        tester,
        decorationWithSuffix,
        TextDirection.ltr,
      );

      expect(textSizeWithSuffixIcon.width, equals(textSizeWithoutIcon.width));
    });

    testWidgets('with prefixIcon and suffixIcon in LTR', (WidgetTester tester) async {
      final Size textSizeWithoutIcon = await measureText(
        tester,
        decorationWithoutIcons,
        TextDirection.ltr,
      );
      final Size textSizeWithIcons = await measureText(
        tester,
        decorationWithAffixes,
        TextDirection.ltr,
      );

      expect(textSizeWithIcons.width, equals(textSizeWithoutIcon.width));
    });

    testWidgets('with prefixIcon in RTL', (WidgetTester tester) async {
      final Size textSizeWithoutIcon = await measureText(
        tester,
        decorationWithoutIcons,
        TextDirection.rtl,
      );
      final Size textSizeWithPrefixIcon = await measureText(
        tester,
        decorationWithPrefix,
        TextDirection.rtl,
      );

      expect(textSizeWithPrefixIcon.width, equals(textSizeWithoutIcon.width));
    });

    testWidgets('with suffixIcon in RTL', (WidgetTester tester) async {
      final Size textSizeWithoutIcon = await measureText(
        tester,
        decorationWithoutIcons,
        TextDirection.rtl,
      );
      final Size textSizeWithSuffixIcon = await measureText(
        tester,
        decorationWithSuffix,
        TextDirection.rtl,
      );

      expect(textSizeWithSuffixIcon.width, equals(textSizeWithoutIcon.width));
    });

    testWidgets('with prefixIcon and suffixIcon in RTL', (WidgetTester tester) async {
      final Size textSizeWithoutIcon = await measureText(
        tester,
        decorationWithoutIcons,
        TextDirection.rtl,
      );
      final Size textSizeWithIcons = await measureText(
        tester,
        decorationWithAffixes,
        TextDirection.rtl,
      );

      expect(textSizeWithIcons.width, equals(textSizeWithoutIcon.width));
    });

    testWidgets('depends on hint width and content width when decorator is empty', (
      WidgetTester tester,
    ) async {
      const InputDecoration decorationWithHint = InputDecoration(
        contentPadding: EdgeInsets.zero,
        hintText: 'Hint',
      );
      const double hintTextWidth = 66.0;
      const double smallContentWidth = 20.0;
      const double largeContentWidth = 80.0;

      await tester.pumpWidget(
        buildInputDecorator(
          decoration: decorationWithHint,
          useIntrinsicWidth: true,
          isEmpty: true,
          child: const SizedBox(width: smallContentWidth),
        ),
      );

      // Decorator width depends on the hint because the hint is larger than the content.
      expect(getDecoratorRect(tester).width, hintTextWidth);

      await tester.pumpWidget(
        buildInputDecorator(
          decoration: decorationWithHint,
          useIntrinsicWidth: true,
          isEmpty: true,
          child: const SizedBox(width: largeContentWidth),
        ),
      );

      // Decorator width depends on the content because the content is larger than the hint.
      expect(getDecoratorRect(tester).width, largeContentWidth);
    });

    // Regression test for https://github.com/flutter/flutter/issues/93337.
    testWidgets(
      'depends on content width when decorator is not empty and maintainHintSize is false',
      (WidgetTester tester) async {
        const InputDecoration decorationWithHint = InputDecoration(
          contentPadding: EdgeInsets.zero,
          hintText: 'Hint',
          maintainHintSize: false,
        );
        const double contentWidth = 20.0;

        await tester.pumpWidget(
          buildInputDecorator(
            decoration: decorationWithHint,
            useIntrinsicWidth: true,
            child: const SizedBox(width: contentWidth),
          ),
        );

        // The hint width is ignored even if larger than the content width.
        expect(getDecoratorRect(tester).width, contentWidth);
      },
    );

    // Regression test for https://github.com/flutter/flutter/issues/93337.
    testWidgets(
      'depends on content width when decorator is not empty and maintainHintSize is true',
      (WidgetTester tester) async {
        const InputDecoration decorationWithHint = InputDecoration(
          contentPadding: EdgeInsets.zero,
          hintText: 'Hint',
        );
        const double contentWidth = 20.0;

        await tester.pumpWidget(
          buildInputDecorator(
            decoration: decorationWithHint,
            useIntrinsicWidth: true,
            child: const SizedBox(width: contentWidth),
          ),
        );

        // The hint width is ignored even if larger than the content width.
        const double hintTextWidth = 66.0;
        expect(getDecoratorRect(tester).width, hintTextWidth);
      },
    );
  });

  testWidgets('Ensure the height of labelStyle remains unchanged when TextField is focused', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/141448.
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: TextField(
            focusNode: focusNode,
            decoration: const InputDecoration(labelText: 'label'),
          ),
        ),
      ),
    );
    final TextStyle beforeStyle = getLabelStyle(tester);
    // Focused.
    focusNode.requestFocus();
    await tester.pump(kTransitionDuration);

    expect(getLabelStyle(tester).height, beforeStyle.height);
  });

  test('InputDecorationTheme.copyWith keeps original iconColor', () async {
    const InputDecorationTheme original = InputDecorationTheme(iconColor: Color(0xDEADBEEF));
    expect(original.iconColor, const Color(0xDEADBEEF));
    expect(original.fillColor, isNot(const Color(0xDEADCAFE)));
    final InputDecorationTheme copy1 = original.copyWith(fillColor: const Color(0xDEADCAFE));
    expect(copy1.iconColor, const Color(0xDEADBEEF));
    expect(copy1.fillColor, const Color(0xDEADCAFE));
    final InputDecorationTheme copy2 = original.copyWith(iconColor: const Color(0xDEADCAFE));
    expect(copy2.iconColor, const Color(0xDEADCAFE));
    expect(copy2.fillColor, isNot(const Color(0xDEADCAFE)));
  });

  test('InputDecorationTheme copyWith, ==, hashCode basics', () {
    expect(const InputDecorationTheme(), const InputDecorationTheme().copyWith());
    expect(const InputDecorationTheme().hashCode, const InputDecorationTheme().copyWith().hashCode);
  });

  test('InputDecorationTheme copyWith correctly copies and replaces values', () {
    const InputDecorationTheme original = InputDecorationTheme(
      focusColor: Colors.orange,
      fillColor: Colors.green,
    );
    final InputDecorationTheme copy = original.copyWith(
      focusColor: Colors.yellow,
      fillColor: Colors.blue,
    );

    expect(original.focusColor, Colors.orange);
    expect(original.fillColor, Colors.green);
    expect(copy.focusColor, Colors.yellow);
    expect(copy.fillColor, Colors.blue);
  });

  test('InputDecorationTheme merge', () {
    const InputDecorationTheme overrideTheme = InputDecorationTheme(
      labelStyle: TextStyle(color: Color(0x000000f0)),
      floatingLabelStyle: TextStyle(color: Color(0x000000f1)),
      helperStyle: TextStyle(color: Color(0x000000f2)),
      helperMaxLines: 1,
      hintStyle: TextStyle(color: Color(0x000000f3)),
      errorStyle: TextStyle(color: Color(0x000000f4)),
      errorMaxLines: 1,
      floatingLabelBehavior: FloatingLabelBehavior.never,
      floatingLabelAlignment: FloatingLabelAlignment.center,
      isDense: true,
      contentPadding: EdgeInsets.all(1.0),
      isCollapsed: true,
      iconColor: Color(0x000000f5),
      prefixStyle: TextStyle(color: Color(0x000000f6)),
      prefixIconColor: Color(0x000000f7),
      suffixStyle: TextStyle(color: Color(0x000000f8)),
      suffixIconColor: Color(0x000000f9),
      counterStyle: TextStyle(color: Color(0x00000f10)),
      filled: true,
      fillColor: Color(0x00000f11),
      activeIndicatorBorder: BorderSide(color: Color(0x00000f12)),
      outlineBorder: BorderSide(color: Color(0x00000f13)),
      focusColor: Color(0x00000f14),
      hoverColor: Color(0x00000f15),
      errorBorder: OutlineInputBorder(borderRadius: BorderRadius.all(Radius.circular(2.0))),
      focusedBorder: OutlineInputBorder(borderSide: BorderSide(color: Color(0x00000f16))),
      focusedErrorBorder: OutlineInputBorder(borderSide: BorderSide(color: Color(0x00000f17))),
      disabledBorder: OutlineInputBorder(borderSide: BorderSide(color: Color(0x00000f18))),
      enabledBorder: OutlineInputBorder(borderSide: BorderSide(color: Color(0x00000f19))),
      border: OutlineInputBorder(borderSide: BorderSide(color: Color(0x00000f20))),
      alignLabelWithHint: true,
      constraints: BoxConstraints(minHeight: 1.0, minWidth: 1.0),
    );

    final InputDecorationTheme inputDecorationTheme = ThemeData().inputDecorationTheme;
    final InputDecorationTheme merged = inputDecorationTheme.merge(overrideTheme);

    expect(merged.labelStyle, overrideTheme.labelStyle);
    expect(merged.floatingLabelStyle, overrideTheme.floatingLabelStyle);
    expect(merged.helperStyle, overrideTheme.helperStyle);
    expect(merged.helperMaxLines, overrideTheme.helperMaxLines);
    expect(merged.hintStyle, overrideTheme.hintStyle);
    expect(merged.errorStyle, overrideTheme.errorStyle);
    expect(merged.errorMaxLines, overrideTheme.errorMaxLines);
    expect(merged.floatingLabelBehavior, isNot(overrideTheme.floatingLabelBehavior));
    expect(merged.floatingLabelAlignment, isNot(overrideTheme.floatingLabelAlignment));
    expect(merged.isDense, isNot(overrideTheme.isDense));
    expect(merged.contentPadding, overrideTheme.contentPadding);
    expect(merged.isCollapsed, isNot(overrideTheme.isCollapsed));
    expect(merged.iconColor, overrideTheme.iconColor);
    expect(merged.prefixStyle, overrideTheme.prefixStyle);
    expect(merged.prefixIconColor, overrideTheme.prefixIconColor);
    expect(merged.suffixStyle, overrideTheme.suffixStyle);
    expect(merged.suffixIconColor, overrideTheme.suffixIconColor);
    expect(merged.counterStyle, overrideTheme.counterStyle);
    expect(merged.filled, isNot(overrideTheme.filled));
    expect(merged.fillColor, overrideTheme.fillColor);
    expect(merged.activeIndicatorBorder, overrideTheme.activeIndicatorBorder);
    expect(merged.outlineBorder, overrideTheme.outlineBorder);
    expect(merged.focusColor, overrideTheme.focusColor);
    expect(merged.hoverColor, overrideTheme.hoverColor);
    expect(merged.errorBorder, overrideTheme.errorBorder);
    expect(merged.focusedBorder, overrideTheme.focusedBorder);
    expect(merged.focusedErrorBorder, overrideTheme.focusedErrorBorder);
    expect(merged.disabledBorder, overrideTheme.disabledBorder);
    expect(merged.enabledBorder, overrideTheme.enabledBorder);
    expect(merged.border, overrideTheme.border);
    expect(merged.alignLabelWithHint, isNot(overrideTheme.alignLabelWithHint));
    expect(merged.constraints, overrideTheme.constraints);
  });

  testWidgets('Prefix IconButton inherits IconButtonTheme', (WidgetTester tester) async {
    const IconData prefixIcon = Icons.person;
    const Color backgroundColor = Color(0xffff0000);
    const Color foregroundColor = Color(0xff00ff00);
    const Color overlayColor = Color(0xff0000ff);
    const Color shadowColor = Color(0xff0ff0ff);
    const double elevation = 4.0;
    final RoundedRectangleBorder shape = RoundedRectangleBorder(
      borderRadius: BorderRadius.circular(10.0),
    );
    final ButtonStyle iconButtonStyle = ButtonStyle(
      backgroundColor: const MaterialStatePropertyAll<Color>(backgroundColor),
      foregroundColor: const MaterialStatePropertyAll<Color>(foregroundColor),
      overlayColor: const MaterialStatePropertyAll<Color>(overlayColor),
      shadowColor: const MaterialStatePropertyAll<Color>(shadowColor),
      elevation: const MaterialStatePropertyAll<double>(elevation),
      shape: MaterialStatePropertyAll<OutlinedBorder>(shape),
    );

    await tester.pumpWidget(
      IconButtonTheme(
        data: IconButtonThemeData(style: iconButtonStyle),
        child: buildInputDecorator(
          decoration: InputDecoration(
            prefixIcon: IconButton(onPressed: () {}, icon: const Icon(prefixIcon)),
          ),
        ),
      ),
    );

    final Finder iconMaterial = find.descendant(
      of: find.widgetWithIcon(IconButton, prefixIcon),
      matching: find.byType(Material),
    );
    final Material material = tester.widget<Material>(iconMaterial);
    expect(material.color, backgroundColor);
    expect(material.shadowColor, shadowColor);
    expect(material.elevation, elevation);
    expect(material.shape, shape);

    expect(getIconStyle(tester, prefixIcon)?.color, foregroundColor);

    final Offset center = tester.getCenter(find.byIcon(prefixIcon));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(getOverlayColor(tester), paints..rect(color: overlayColor));
  });

  testWidgets('Suffix IconButton inherits IconButtonTheme', (WidgetTester tester) async {
    const IconData suffixIcon = Icons.delete;
    const Color backgroundColor = Color(0xffff0000);
    const Color foregroundColor = Color(0xff00ff00);
    const Color overlayColor = Color(0xff0000ff);
    const Color shadowColor = Color(0xff0ff0ff);
    const double elevation = 4.0;
    final RoundedRectangleBorder shape = RoundedRectangleBorder(
      borderRadius: BorderRadius.circular(10.0),
    );
    final ButtonStyle iconButtonStyle = ButtonStyle(
      backgroundColor: const MaterialStatePropertyAll<Color>(backgroundColor),
      foregroundColor: const MaterialStatePropertyAll<Color>(foregroundColor),
      overlayColor: const MaterialStatePropertyAll<Color>(overlayColor),
      shadowColor: const MaterialStatePropertyAll<Color>(shadowColor),
      elevation: const MaterialStatePropertyAll<double>(elevation),
      shape: MaterialStatePropertyAll<OutlinedBorder>(shape),
    );

    await tester.pumpWidget(
      IconButtonTheme(
        data: IconButtonThemeData(style: iconButtonStyle),
        child: buildInputDecorator(
          decoration: InputDecoration(
            suffixIcon: IconButton(onPressed: () {}, icon: const Icon(suffixIcon)),
          ),
        ),
      ),
    );

    final Finder iconMaterial = find.descendant(
      of: find.widgetWithIcon(IconButton, suffixIcon),
      matching: find.byType(Material),
    );
    final Material material = tester.widget<Material>(iconMaterial);
    expect(material.color, backgroundColor);
    expect(material.shadowColor, shadowColor);
    expect(material.elevation, elevation);
    expect(material.shape, shape);

    expect(getIconStyle(tester, suffixIcon)?.color, foregroundColor);

    final Offset center = tester.getCenter(find.byIcon(suffixIcon));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(getOverlayColor(tester), paints..rect(color: overlayColor));
  });

  testWidgets('Prefix IconButton color respects IconButtonTheme foreground color states', (
    WidgetTester tester,
  ) async {
    const IconData prefixIcon = Icons.person;
    const Color iconErrorColor = Color(0xffff0000);
    const Color iconColor = Color(0xff00ff00);
    final ButtonStyle iconButtonStyle = ButtonStyle(
      foregroundColor: MaterialStateProperty.resolveWith<Color?>((Set<MaterialState> states) {
        if (states.contains(MaterialState.error)) {
          return iconErrorColor;
        }
        return iconColor;
      }),
    );

    // Test the prefix IconButton color when there is an error text.
    await tester.pumpWidget(
      buildInputDecorator(
        iconButtonTheme: IconButtonThemeData(style: iconButtonStyle),
        decoration: InputDecoration(
          errorText: 'error',
          prefixIcon: IconButton(onPressed: () {}, icon: const Icon(prefixIcon)),
        ),
      ),
    );

    expect(getIconStyle(tester, prefixIcon)?.color, iconErrorColor);

    // Test the prefix IconButton color when there is no error text.
    await tester.pumpWidget(
      buildInputDecorator(
        iconButtonTheme: IconButtonThemeData(style: iconButtonStyle),
        decoration: InputDecoration(
          prefixIcon: IconButton(onPressed: () {}, icon: const Icon(prefixIcon)),
        ),
      ),
    );
    await tester.pump();

    expect(getIconStyle(tester, prefixIcon)?.color, iconColor);
  });

  testWidgets('Suffix IconButton color respects IconButtonTheme foreground color states', (
    WidgetTester tester,
  ) async {
    const IconData suffixIcon = Icons.search;
    const Color iconErrorColor = Color(0xffff0000);
    const Color iconColor = Color(0xff00ff00);
    final ButtonStyle iconButtonStyle = ButtonStyle(
      foregroundColor: MaterialStateProperty.resolveWith<Color?>((Set<MaterialState> states) {
        if (states.contains(MaterialState.error)) {
          return iconErrorColor;
        }
        return iconColor;
      }),
    );

    // Test the prefix IconButton color when there is an error text.
    await tester.pumpWidget(
      buildInputDecorator(
        iconButtonTheme: IconButtonThemeData(style: iconButtonStyle),
        decoration: InputDecoration(
          errorText: 'error',
          suffixIcon: IconButton(onPressed: () {}, icon: const Icon(suffixIcon)),
        ),
      ),
    );

    expect(getIconStyle(tester, suffixIcon)?.color, iconErrorColor);

    // Test the prefix IconButton color when there is no error text.
    await tester.pumpWidget(
      buildInputDecorator(
        iconButtonTheme: IconButtonThemeData(style: iconButtonStyle),
        decoration: InputDecoration(
          suffixIcon: IconButton(onPressed: () {}, icon: const Icon(suffixIcon)),
        ),
      ),
    );
    await tester.pump();

    expect(getIconStyle(tester, suffixIcon)?.color, iconColor);
  });

  group('Material2', () {
    // These tests are only relevant for Material 2. Once Material 2
    // support is deprecated and the APIs are removed, these tests
    // can be deleted.

    Widget buildInputDecoratorM2({
      InputDecoration decoration = const InputDecoration(),
      ThemeData? theme,
      InputDecorationTheme? inputDecorationTheme,
      TextDirection textDirection = TextDirection.ltr,
      bool expands = false,
      bool isEmpty = false,
      bool isFocused = false,
      bool isHovering = false,
      bool useIntrinsicWidth = false,
      TextStyle? baseStyle,
      TextAlignVertical? textAlignVertical,
      VisualDensity? visualDensity,
      Widget child = const Text('text', style: TextStyle(fontSize: 16.0)),
    }) {
      Widget widget = InputDecorator(
        expands: expands,
        decoration: decoration,
        isEmpty: isEmpty,
        isFocused: isFocused,
        isHovering: isHovering,
        baseStyle: baseStyle,
        textAlignVertical: textAlignVertical,
        child: child,
      );

      if (useIntrinsicWidth) {
        widget = IntrinsicWidth(child: widget);
      }

      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Builder(
            builder: (BuildContext context) {
              return Theme(
                data: (theme ?? Theme.of(context)).copyWith(
                  inputDecorationTheme: inputDecorationTheme,
                  visualDensity: visualDensity,
                  textTheme: const TextTheme(bodyLarge: TextStyle(fontSize: 16.0)),
                ),
                child: Align(
                  alignment: Alignment.topLeft,
                  child: Directionality(textDirection: textDirection, child: widget),
                ),
              );
            },
          ),
        ),
      );
    }

    testWidgets('InputDecorator input/label text layout', (WidgetTester tester) async {
      // The label appears above the input text
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: const InputDecoration(labelText: 'label'),
        ),
      );
      await tester.pumpAndSettle();

      // Overall height for this InputDecorator is 56dps:
      //   12 - top padding
      //   12 - floating label (font size 16dps * 0.75 = 12)
      //    4 - floating label / input text gap
      //   16 - input text (font size 16dps)
      //   12 - bottom padding

      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(tester.getTopLeft(find.text('text')).dy, 28.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 44.0);
      expect(tester.getTopLeft(find.text('label')).dy, 12.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 24.0);
      expect(getBorderBottom(tester), 56.0);
      expect(getBorderWeight(tester), 1.0);

      // The label appears within the input when there is no text content
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          // isFocused: false (default)
          decoration: const InputDecoration(labelText: 'label'),
        ),
      );
      await tester.pumpAndSettle();

      expect(tester.getTopLeft(find.text('label')).dy, 20.0);

      // The label appears above the input text when there is no content and floatingLabelBehavior is always
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          // isFocused: false (default)
          decoration: const InputDecoration(
            labelText: 'label',
            floatingLabelBehavior: FloatingLabelBehavior.always,
          ),
        ),
      );
      await tester.pumpAndSettle();

      expect(tester.getTopLeft(find.text('label')).dy, 12.0);

      // The label appears within the input text when there is content and floatingLabelBehavior is never
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isFocused: false (default)
          decoration: const InputDecoration(
            labelText: 'label',
            floatingLabelBehavior: FloatingLabelBehavior.never,
          ),
        ),
      );
      await tester.pumpAndSettle();

      expect(tester.getTopLeft(find.text('label')).dy, 20.0);

      // isFocused: true increases the border's weight from 1.0 to 2.0
      // but does not change the overall height.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          isFocused: true,
          decoration: const InputDecoration(labelText: 'label'),
        ),
      );
      await tester.pumpAndSettle();
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(tester.getTopLeft(find.text('text')).dy, 28.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 44.0);
      expect(tester.getTopLeft(find.text('label')).dy, 12.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 24.0);
      expect(getBorderBottom(tester), 56.0);
      expect(getBorderWeight(tester), 2.0);

      // isEmpty: true causes the label to be aligned with the input text
      await tester.pumpWidget(
        buildInputDecoratorM2(isEmpty: true, decoration: const InputDecoration(labelText: 'label')),
      );

      // The label animates downwards from it's initial position
      // above the input text. The animation's duration is 167ms.
      {
        await tester.pump(const Duration(milliseconds: 50));
        final double labelY50ms = tester.getTopLeft(find.text('label')).dy;
        expect(labelY50ms, inExclusiveRange(12.0, 20.0));
        await tester.pump(const Duration(milliseconds: 50));
        final double labelY100ms = tester.getTopLeft(find.text('label')).dy;
        expect(labelY100ms, inExclusiveRange(labelY50ms, 20.0));
      }
      await tester.pumpAndSettle();
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(tester.getTopLeft(find.text('text')).dy, 28.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 44.0);
      expect(tester.getTopLeft(find.text('label')).dy, 20.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 36.0);
      expect(getBorderBottom(tester), 56.0);
      expect(getBorderWeight(tester), 1.0);

      // isFocused: true causes the label to move back up above the input text.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          isFocused: true,
          decoration: const InputDecoration(labelText: 'label'),
        ),
      );

      // The label animates upwards from it's initial position
      // above the input text. The animation's duration is 167ms.
      await tester.pump(const Duration(milliseconds: 50));
      final double labelY50ms = tester.getTopLeft(find.text('label')).dy;
      expect(labelY50ms, inExclusiveRange(12.0, 28.0));
      await tester.pump(const Duration(milliseconds: 50));
      final double labelY100ms = tester.getTopLeft(find.text('label')).dy;
      expect(labelY100ms, inExclusiveRange(12.0, labelY50ms));

      await tester.pumpAndSettle();
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(tester.getTopLeft(find.text('text')).dy, 28.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 44.0);
      expect(tester.getTopLeft(find.text('label')).dy, 12.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 24.0);
      expect(getBorderBottom(tester), 56.0);
      expect(getBorderWeight(tester), 2.0);

      // enabled: false produces a hairline border if filled: false (the default)
      // The widget's size and layout is the same as for enabled: true.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          decoration: const InputDecoration(labelText: 'label', enabled: false),
        ),
      );
      await tester.pumpAndSettle();
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(tester.getTopLeft(find.text('text')).dy, 28.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 44.0);
      expect(tester.getTopLeft(find.text('label')).dy, 20.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 36.0);
      expect(getBorderWeight(tester), 0.0);

      // enabled: false produces a transparent border if filled: true.
      // The widget's size and layout is the same as for enabled: true.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          decoration: const InputDecoration(labelText: 'label', enabled: false, filled: true),
        ),
      );
      await tester.pumpAndSettle();
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(tester.getTopLeft(find.text('text')).dy, 28.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 44.0);
      expect(tester.getTopLeft(find.text('label')).dy, 20.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 36.0);
      expect(getBorderColor(tester), Colors.transparent);

      // alignLabelWithHint: true positions the label at the text baseline,
      // aligned with the hint.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          decoration: const InputDecoration(
            labelText: 'label',
            alignLabelWithHint: true,
            hintText: 'hint',
          ),
        ),
      );
      await tester.pumpAndSettle();
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(tester.getTopLeft(find.text('label')).dy, tester.getTopLeft(find.text('hint')).dy);
      expect(
        tester.getBottomLeft(find.text('label')).dy,
        tester.getBottomLeft(find.text('hint')).dy,
      );
    });

    testWidgets('InputDecorator input/label widget layout', (WidgetTester tester) async {
      const Key key = Key('l');

      // The label appears above the input text.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: const InputDecoration(
            label: Text.rich(
              TextSpan(
                children: <InlineSpan>[
                  TextSpan(text: 'label'),
                  WidgetSpan(child: Text('*', style: TextStyle(color: Colors.red))),
                ],
              ),
              key: key,
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();

      // Overall height for this InputDecorator is 56dps:
      //   12 - top padding
      //   12 - floating label (font size 16dps * 0.75 = 12)
      //    4 - floating label / input text gap
      //   16 - input text (font size 16dps)
      //   12 - bottom padding

      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(tester.getTopLeft(find.text('text')).dy, 28.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 44.0);
      expect(tester.getTopLeft(find.byKey(key)).dy, 12.0);
      expect(tester.getBottomLeft(find.byKey(key)).dy, 24.0);
      expect(getBorderBottom(tester), 56.0);
      expect(getBorderWeight(tester), 1.0);

      // The label appears within the input when there is no text content.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          // isFocused: false (default)
          decoration: const InputDecoration(
            label: Text.rich(
              TextSpan(
                children: <InlineSpan>[
                  TextSpan(text: 'label'),
                  WidgetSpan(child: Text('*', style: TextStyle(color: Colors.red))),
                ],
              ),
              key: key,
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();

      expect(tester.getTopLeft(find.byKey(key)).dy, 20.0);

      // The label appears above the input text when there is no content and the
      // floatingLabelBehavior is set to always.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          // isFocused: false (default)
          decoration: const InputDecoration(
            label: Text.rich(
              TextSpan(
                children: <InlineSpan>[
                  TextSpan(text: 'label'),
                  WidgetSpan(child: Text('*', style: TextStyle(color: Colors.red))),
                ],
              ),
              key: key,
            ),
            floatingLabelBehavior: FloatingLabelBehavior.always,
          ),
        ),
      );
      await tester.pumpAndSettle();

      expect(tester.getTopLeft(find.byKey(key)).dy, 12.0);

      // The label appears within the input text when there is content and
      // the floatingLabelBehavior is set to never.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isFocused: false (default)
          decoration: const InputDecoration(
            label: Text.rich(
              TextSpan(
                children: <InlineSpan>[
                  TextSpan(text: 'label'),
                  WidgetSpan(child: Text('*', style: TextStyle(color: Colors.red))),
                ],
              ),
              key: key,
            ),
            floatingLabelBehavior: FloatingLabelBehavior.never,
          ),
        ),
      );
      await tester.pumpAndSettle();

      expect(tester.getTopLeft(find.byKey(key)).dy, 20.0);

      // Overall height for this InputDecorator is 56dps:
      //   12 - top padding
      //   12 - floating label (font size 16dps * 0.75 = 12)
      //    4 - floating label / input text gap
      //   16 - input text (font size 16dps)
      //   12 - bottom padding

      expect(tester.getTopLeft(find.byKey(key)).dy, 20.0);

      // isFocused: true increases the border's weight from 1.0 to 2.0
      // but does not change the overall height.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          isFocused: true,
          decoration: const InputDecoration(
            label: Text.rich(
              TextSpan(
                children: <InlineSpan>[
                  TextSpan(text: 'label'),
                  WidgetSpan(child: Text('*', style: TextStyle(color: Colors.red))),
                ],
              ),
              key: key,
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(tester.getTopLeft(find.text('text')).dy, 28.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 44.0);
      expect(tester.getTopLeft(find.byKey(key)).dy, 12.0);
      expect(tester.getBottomLeft(find.byKey(key)).dy, 24.0);
      expect(getBorderBottom(tester), 56.0);
      expect(getBorderWeight(tester), 2.0);

      // isEmpty: true causes the label to be aligned with the input text.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          decoration: const InputDecoration(
            label: Text.rich(
              TextSpan(
                children: <InlineSpan>[
                  TextSpan(text: 'label'),
                  WidgetSpan(child: Text('*', style: TextStyle(color: Colors.red))),
                ],
              ),
              key: key,
            ),
          ),
        ),
      );

      // The label animates downwards from it's initial position
      // above the input text. The animation's duration is 167ms.
      await tester.pump(const Duration(milliseconds: 50));
      final double labelY50ms = tester.getTopLeft(find.byKey(key)).dy;
      expect(labelY50ms, inExclusiveRange(12.0, 20.0));
      await tester.pump(const Duration(milliseconds: 50));
      final double labelY100ms = tester.getTopLeft(find.byKey(key)).dy;
      expect(labelY100ms, inExclusiveRange(labelY50ms, 20.0));

      await tester.pumpAndSettle();
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(tester.getTopLeft(find.text('text')).dy, 28.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 44.0);
      expect(tester.getTopLeft(find.byKey(key)).dy, 20.0);
      expect(tester.getBottomLeft(find.byKey(key)).dy, 36.0);
      expect(getBorderBottom(tester), 56.0);
      expect(getBorderWeight(tester), 1.0);

      // isFocused: true causes the label to move back up above the input text.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          isFocused: true,
          decoration: const InputDecoration(
            label: Text.rich(
              TextSpan(
                children: <InlineSpan>[
                  TextSpan(text: 'label'),
                  WidgetSpan(child: Text('*', style: TextStyle(color: Colors.red))),
                ],
              ),
              key: key,
            ),
          ),
        ),
      );

      // The label animates upwards from it's initial position
      // above the input text. The animation's duration is 167ms.
      {
        await tester.pump(const Duration(milliseconds: 50));
        final double labelY50ms = tester.getTopLeft(find.byKey(key)).dy;
        expect(labelY50ms, inExclusiveRange(12.0, 28.0));
        await tester.pump(const Duration(milliseconds: 50));
        final double labelY100ms = tester.getTopLeft(find.byKey(key)).dy;
        expect(labelY100ms, inExclusiveRange(12.0, labelY50ms));
      }

      await tester.pumpAndSettle();
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(tester.getTopLeft(find.text('text')).dy, 28.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 44.0);
      expect(tester.getTopLeft(find.byKey(key)).dy, 12.0);
      expect(tester.getBottomLeft(find.byKey(key)).dy, 24.0);
      expect(getBorderBottom(tester), 56.0);
      expect(getBorderWeight(tester), 2.0);

      // enabled: false produces a hairline border if filled: false (the default)
      // The widget's size and layout is the same as for enabled: true.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          decoration: const InputDecoration(
            label: Text.rich(
              TextSpan(
                children: <InlineSpan>[
                  TextSpan(text: 'label'),
                  WidgetSpan(child: Text('*', style: TextStyle(color: Colors.red))),
                ],
              ),
              key: key,
            ),
            enabled: false,
          ),
        ),
      );
      await tester.pumpAndSettle();
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(tester.getTopLeft(find.text('text')).dy, 28.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 44.0);
      expect(tester.getTopLeft(find.byKey(key)).dy, 20.0);
      expect(tester.getBottomLeft(find.byKey(key)).dy, 36.0);
      expect(getBorderWeight(tester), 0.0);

      // enabled: false produces a transparent border if filled: true.
      // The widget's size and layout is the same as for enabled: true.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          decoration: const InputDecoration(
            label: Text.rich(
              TextSpan(
                children: <InlineSpan>[
                  TextSpan(text: 'label'),
                  WidgetSpan(child: Text('*', style: TextStyle(color: Colors.red))),
                ],
              ),
              key: key,
            ),
            enabled: false,
            filled: true,
          ),
        ),
      );
      await tester.pumpAndSettle();
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(tester.getTopLeft(find.text('text')).dy, 28.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 44.0);
      expect(tester.getTopLeft(find.byKey(key)).dy, 20.0);
      expect(tester.getBottomLeft(find.byKey(key)).dy, 36.0);
      expect(getBorderColor(tester), Colors.transparent);

      // alignLabelWithHint: true positions the label at the text baseline,
      // aligned with the hint.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          decoration: const InputDecoration(
            label: Text.rich(
              TextSpan(
                children: <InlineSpan>[
                  TextSpan(text: 'label'),
                  WidgetSpan(child: Text('*', style: TextStyle(color: Colors.red))),
                ],
              ),
              key: key,
            ),
            alignLabelWithHint: true,
            hintText: 'hint',
          ),
        ),
      );
      await tester.pumpAndSettle();
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(tester.getTopLeft(find.byKey(key)).dy, tester.getTopLeft(find.text('hint')).dy);
      expect(tester.getBottomLeft(find.byKey(key)).dy, tester.getBottomLeft(find.text('hint')).dy);
    });

    testWidgets('InputDecorator floating label animation duration and curve', (
      WidgetTester tester,
    ) async {
      Future<void> pumpInputDecorator({required bool isFocused}) async {
        return tester.pumpWidget(
          buildInputDecoratorM2(
            isEmpty: true,
            isFocused: isFocused,
            decoration: const InputDecoration(
              labelText: 'label',
              floatingLabelBehavior: FloatingLabelBehavior.auto,
            ),
          ),
        );
      }

      await pumpInputDecorator(isFocused: false);
      expect(tester.getTopLeft(find.text('label')).dy, 20.0);

      // The label animates upwards and scales down.
      // The animation duration is 167ms and the curve is fastOutSlowIn.
      await pumpInputDecorator(isFocused: true);
      await tester.pump(const Duration(milliseconds: 42));
      expect(tester.getTopLeft(find.text('label')).dy, closeTo(18.06, 0.5));
      await tester.pump(const Duration(milliseconds: 42));
      expect(tester.getTopLeft(find.text('label')).dy, closeTo(13.78, 0.5));
      await tester.pump(const Duration(milliseconds: 42));
      expect(tester.getTopLeft(find.text('label')).dy, closeTo(12.31, 0.5));
      await tester.pump(const Duration(milliseconds: 41));
      expect(tester.getTopLeft(find.text('label')).dy, 12.0);

      // If the animation changes direction without first reaching the
      // AnimationStatus.completed or AnimationStatus.dismissed status,
      // the CurvedAnimation stays on the same curve in the opposite direction.
      // The pumpAndSettle is used to prevent this behavior.
      await tester.pumpAndSettle();

      // The label animates downwards and scales up.
      // The animation duration is 167ms and the curve is fastOutSlowIn.
      await pumpInputDecorator(isFocused: false);
      await tester.pump(const Duration(milliseconds: 42));
      expect(tester.getTopLeft(find.text('label')).dy, closeTo(13.94, 0.5));
      await tester.pump(const Duration(milliseconds: 42));
      expect(tester.getTopLeft(find.text('label')).dy, closeTo(18.22, 0.5));
      await tester.pump(const Duration(milliseconds: 42));
      expect(tester.getTopLeft(find.text('label')).dy, closeTo(19.69, 0.5));
      await tester.pump(const Duration(milliseconds: 41));
      expect(tester.getTopLeft(find.text('label')).dy, 20.0);
    });

    group('alignLabelWithHint', () {
      group('expands false', () {
        testWidgets('multiline TextField no-strut', (WidgetTester tester) async {
          const String text = 'text';
          final FocusNode focusNode = FocusNode();
          final TextEditingController controller = TextEditingController();
          addTearDown(() {
            focusNode.dispose();
            controller.dispose();
          });

          Widget buildFrame(bool alignLabelWithHint) {
            return MaterialApp(
              theme: ThemeData(useMaterial3: false),
              home: Material(
                child: Directionality(
                  textDirection: TextDirection.ltr,
                  child: TextField(
                    controller: controller,
                    focusNode: focusNode,
                    maxLines: 8,
                    decoration: InputDecoration(
                      labelText: 'label',
                      alignLabelWithHint: alignLabelWithHint,
                      hintText: 'hint',
                    ),
                    strutStyle: StrutStyle.disabled,
                  ),
                ),
              ),
            );
          }

          // alignLabelWithHint: false centers the label in the TextField.
          await tester.pumpWidget(buildFrame(false));
          await tester.pumpAndSettle();
          expect(tester.getTopLeft(find.text('label')).dy, 76.0);
          expect(tester.getBottomLeft(find.text('label')).dy, 92.0);

          // Entering text still happens at the top.
          await tester.enterText(find.byType(TextField), text);
          expect(tester.getTopLeft(find.text(text)).dy, 28.0);
          controller.clear();
          focusNode.unfocus();

          // alignLabelWithHint: true aligns the label with the hint.
          await tester.pumpWidget(buildFrame(true));
          await tester.pumpAndSettle();
          expect(tester.getTopLeft(find.text('label')).dy, tester.getTopLeft(find.text('hint')).dy);
          expect(
            tester.getBottomLeft(find.text('label')).dy,
            tester.getBottomLeft(find.text('hint')).dy,
          );

          // Entering text still happens at the top.
          await tester.enterText(find.byType(TextField), text);
          expect(tester.getTopLeft(find.text(text)).dy, 28.0);
          controller.clear();
          focusNode.unfocus();
        });

        testWidgets('multiline TextField', (WidgetTester tester) async {
          const String text = 'text';
          final FocusNode focusNode = FocusNode();
          final TextEditingController controller = TextEditingController();
          addTearDown(() {
            focusNode.dispose();
            controller.dispose();
          });
          Widget buildFrame(bool alignLabelWithHint) {
            return MaterialApp(
              theme: ThemeData(useMaterial3: false),
              home: Material(
                child: Directionality(
                  textDirection: TextDirection.ltr,
                  child: TextField(
                    controller: controller,
                    focusNode: focusNode,
                    maxLines: 8,
                    decoration: InputDecoration(
                      labelText: 'label',
                      alignLabelWithHint: alignLabelWithHint,
                      hintText: 'hint',
                    ),
                  ),
                ),
              ),
            );
          }

          // alignLabelWithHint: false centers the label in the TextField.
          await tester.pumpWidget(buildFrame(false));
          await tester.pumpAndSettle();
          expect(tester.getTopLeft(find.text('label')).dy, 76.0);
          expect(tester.getBottomLeft(find.text('label')).dy, 92.0);

          // Entering text still happens at the top.
          await tester.enterText(find.byType(InputDecorator), text);
          expect(tester.getTopLeft(find.text(text)).dy, 28.0);
          controller.clear();
          focusNode.unfocus();

          // alignLabelWithHint: true aligns the label with the hint.
          await tester.pumpWidget(buildFrame(true));
          await tester.pumpAndSettle();
          expect(tester.getTopLeft(find.text('label')).dy, tester.getTopLeft(find.text('hint')).dy);
          expect(
            tester.getBottomLeft(find.text('label')).dy,
            tester.getBottomLeft(find.text('hint')).dy,
          );

          // Entering text still happens at the top.
          await tester.enterText(find.byType(InputDecorator), text);
          expect(tester.getTopLeft(find.text(text)).dy, 28.0);
          controller.clear();
          focusNode.unfocus();
        });
      });

      group('expands true', () {
        testWidgets('multiline TextField', (WidgetTester tester) async {
          const String text = 'text';
          final FocusNode focusNode = FocusNode();
          addTearDown(focusNode.dispose);
          final TextEditingController controller = TextEditingController();
          addTearDown(controller.dispose);

          Widget buildFrame(bool alignLabelWithHint) {
            return MaterialApp(
              theme: ThemeData(useMaterial3: false),
              home: Material(
                child: Directionality(
                  textDirection: TextDirection.ltr,
                  child: TextField(
                    controller: controller,
                    focusNode: focusNode,
                    maxLines: null,
                    expands: true,
                    decoration: InputDecoration(
                      labelText: 'label',
                      alignLabelWithHint: alignLabelWithHint,
                      hintText: 'hint',
                    ),
                  ),
                ),
              ),
            );
          }

          // alignLabelWithHint: false centers the label in the TextField.
          await tester.pumpWidget(buildFrame(false));
          await tester.pumpAndSettle();
          expect(tester.getTopLeft(find.text('label')).dy, 292.0);
          expect(tester.getBottomLeft(find.text('label')).dy, 308.0);

          // Entering text still happens at the top.
          await tester.enterText(find.byType(InputDecorator), text);
          expect(tester.getTopLeft(find.text(text)).dy, 28.0);
          controller.clear();
          focusNode.unfocus();

          // alignLabelWithHint: true aligns the label with the hint at the top.
          await tester.pumpWidget(buildFrame(true));
          await tester.pumpAndSettle();
          expect(tester.getTopLeft(find.text('label')).dy, 28.0);
          expect(tester.getTopLeft(find.text('label')).dy, tester.getTopLeft(find.text('hint')).dy);
          expect(
            tester.getBottomLeft(find.text('label')).dy,
            tester.getBottomLeft(find.text('hint')).dy,
          );

          // Entering text still happens at the top.
          await tester.enterText(find.byType(InputDecorator), text);
          expect(tester.getTopLeft(find.text(text)).dy, 28.0);
          controller.clear();
          focusNode.unfocus();
        });

        testWidgets('multiline TextField with outline border', (WidgetTester tester) async {
          const String text = 'text';
          final FocusNode focusNode = FocusNode();
          addTearDown(focusNode.dispose);
          final TextEditingController controller = TextEditingController();
          addTearDown(controller.dispose);

          Widget buildFrame(bool alignLabelWithHint) {
            return MaterialApp(
              theme: ThemeData(useMaterial3: false),
              home: Material(
                child: Directionality(
                  textDirection: TextDirection.ltr,
                  child: TextField(
                    controller: controller,
                    focusNode: focusNode,
                    maxLines: null,
                    expands: true,
                    decoration: InputDecoration(
                      labelText: 'label',
                      alignLabelWithHint: alignLabelWithHint,
                      hintText: 'hint',
                      border: const OutlineInputBorder(borderRadius: BorderRadius.zero),
                    ),
                  ),
                ),
              ),
            );
          }

          // alignLabelWithHint: false centers the label in the TextField.
          await tester.pumpWidget(buildFrame(false));
          await tester.pumpAndSettle();
          expect(tester.getTopLeft(find.text('label')).dy, 292.0);
          expect(tester.getBottomLeft(find.text('label')).dy, 308.0);

          // Entering text happens in the center as well.
          await tester.enterText(find.byType(InputDecorator), text);
          expect(tester.getTopLeft(find.text(text)).dy, 292.0);
          controller.clear();
          focusNode.unfocus();

          // alignLabelWithHint: true aligns keeps the label in the center because
          // that's where the hint is.
          await tester.pumpWidget(buildFrame(true));
          await tester.pumpAndSettle();
          expect(tester.getTopLeft(find.text('label')).dy, 292.0);
          expect(tester.getTopLeft(find.text('label')).dy, tester.getTopLeft(find.text('hint')).dy);
          expect(
            tester.getBottomLeft(find.text('label')).dy,
            tester.getBottomLeft(find.text('hint')).dy,
          );

          // Entering text still happens in the center.
          await tester.enterText(find.byType(InputDecorator), text);
          expect(tester.getTopLeft(find.text(text)).dy, 292.0);
          controller.clear();
          focusNode.unfocus();
        });
      });
    });

    // Overall height for this InputDecorator is 40.0dps
    //   12 - top padding
    //   16 - input text (font size 16dps)
    //   12 - bottom padding
    testWidgets('InputDecorator input/hint layout', (WidgetTester tester) async {
      // The hint aligns with the input text
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          // isFocused: false (default)
          decoration: const InputDecoration(hintText: 'hint'),
        ),
      );

      expect(
        tester.getSize(find.byType(InputDecorator)),
        const Size(800.0, kMinInteractiveDimension),
      );
      expect(tester.getTopLeft(find.text('text')).dy, 16.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 32.0);
      expect(tester.getTopLeft(find.text('hint')).dy, 16.0);
      expect(tester.getBottomLeft(find.text('hint')).dy, 32.0);
      expect(getBorderBottom(tester), 48.0);
      expect(getBorderWeight(tester), 1.0);

      expect(tester.getSize(find.text('hint')).width, tester.getSize(find.text('text')).width);
    });

    testWidgets('InputDecorator input/label/hint layout', (WidgetTester tester) async {
      // Label is visible, hint is not (opacity 0.0).
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          // isFocused: false (default)
          decoration: const InputDecoration(labelText: 'label', hintText: 'hint'),
        ),
      );

      // Overall height for this InputDecorator is 56dps. When the
      // label is "floating" (empty input or no focus) the layout is:
      //
      //   12 - top padding
      //   12 - floating label (font size 16dps * 0.75 = 12)
      //    4 - floating label / input text gap
      //   16 - input text (font size 16dps)
      //   12 - bottom padding
      //
      // When the label is not floating, it's vertically centered.
      //
      //   20 - top padding
      //   16 - label (font size 16dps)
      //   20 - bottom padding (empty input text still appears here)

      // The label is not floating so it's vertically centered.
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(tester.getTopLeft(find.text('text')).dy, 28.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 44.0);
      expect(tester.getTopLeft(find.text('label')).dy, 20.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 36.0);
      expect(getOpacity(tester, 'hint'), 0.0);
      expect(getBorderBottom(tester), 56.0);
      expect(getBorderWeight(tester), 1.0);

      // Label moves upwards, hint is visible (opacity 1.0).
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          isFocused: true,
          decoration: const InputDecoration(labelText: 'label', hintText: 'hint'),
        ),
      );

      // The hint's opacity animates from 0.0 to 1.0.
      // The animation's default duration is 20ms.
      {
        await tester.pump(const Duration(milliseconds: 9));
        final double hintOpacity9ms = getOpacity(tester, 'hint');
        expect(hintOpacity9ms, inExclusiveRange(0.0, 1.0));
        await tester.pump(const Duration(milliseconds: 9));
        final double hintOpacity18ms = getOpacity(tester, 'hint');
        expect(hintOpacity18ms, inExclusiveRange(hintOpacity9ms, 1.0));
      }

      await tester.pumpAndSettle();
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(tester.getTopLeft(find.text('text')).dy, 28.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 44.0);
      expect(tester.getTopLeft(find.text('label')).dy, 12.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 24.0);
      expect(tester.getTopLeft(find.text('hint')).dy, 28.0);
      expect(tester.getBottomLeft(find.text('hint')).dy, 44.0);
      expect(getOpacity(tester, 'hint'), 1.0);
      expect(getBorderBottom(tester), 56.0);
      expect(getBorderWeight(tester), 2.0);

      await tester.pumpWidget(
        buildInputDecoratorM2(
          isFocused: true,
          decoration: const InputDecoration(labelText: 'label', hintText: 'hint'),
        ),
      );

      // The hint's opacity animates from 1.0 to 0.0.
      // The animation's default duration is 20ms.
      {
        await tester.pump(const Duration(milliseconds: 9));
        final double hintOpacity9ms = getOpacity(tester, 'hint');
        expect(hintOpacity9ms, inExclusiveRange(0.0, 1.0));
        await tester.pump(const Duration(milliseconds: 9));
        final double hintOpacity18ms = getOpacity(tester, 'hint');
        expect(hintOpacity18ms, inExclusiveRange(0.0, hintOpacity9ms));
      }

      await tester.pumpAndSettle();
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(tester.getTopLeft(find.text('text')).dy, 28.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 44.0);
      expect(tester.getTopLeft(find.text('label')).dy, 12.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 24.0);
      expect(tester.getTopLeft(find.text('hint')).dy, 28.0);
      expect(tester.getBottomLeft(find.text('hint')).dy, 44.0);
      expect(getOpacity(tester, 'hint'), 0.0);
      expect(getBorderBottom(tester), 56.0);
      expect(getBorderWeight(tester), 2.0);
    });

    testWidgets('InputDecorator input/label/hint dense layout', (WidgetTester tester) async {
      // Label is visible, hint is not (opacity 0.0).
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          // isFocused: false (default)
          decoration: const InputDecoration(labelText: 'label', hintText: 'hint', isDense: true),
        ),
      );

      // Overall height for this InputDecorator is 48dps. When the
      // label is "floating" (empty input or no focus) the layout is:
      //
      //    8 - top padding
      //   12 - floating label (font size 16dps * 0.75 = 12)
      //    4 - floating label / input text gap
      //   16 - input text (font size 16dps)
      //    8 - bottom padding
      //
      // When the label is not floating, it's vertically centered.
      //
      //   16 - top padding
      //   16 - label (font size 16dps)
      //   16 - bottom padding (empty input text still appears here)

      // The label is not floating so it's vertically centered.
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 48.0));
      expect(tester.getTopLeft(find.text('text')).dy, 24.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 40.0);
      expect(tester.getTopLeft(find.text('label')).dy, 16.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 32.0);
      expect(getOpacity(tester, 'hint'), 0.0);
      expect(getBorderBottom(tester), 48.0);
      expect(getBorderWeight(tester), 1.0);

      // Label is visible, hint is not (opacity 0.0).
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          isFocused: true,
          decoration: const InputDecoration(labelText: 'label', hintText: 'hint', isDense: true),
        ),
      );
      await tester.pumpAndSettle();
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 48.0));
      expect(tester.getTopLeft(find.text('text')).dy, 24.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 40.0);
      expect(tester.getTopLeft(find.text('label')).dy, 8.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 20.0);
      expect(getOpacity(tester, 'hint'), 1.0);
      expect(getBorderBottom(tester), 48.0);
      expect(getBorderWeight(tester), 2.0);
    });

    testWidgets('InputDecorator default hint animation duration', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          decoration: const InputDecoration(labelText: 'label', hintText: 'hint'),
        ),
      );

      // The hint is not visible (opacity 0.0).
      expect(getOpacity(tester, 'hint'), 0.0);

      // Focus to show the hint.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          isFocused: true,
          decoration: const InputDecoration(labelText: 'label', hintText: 'hint'),
        ),
      );

      // The hint's opacity animates from 0.0 to 1.0.
      // The animation's default duration is 20ms.
      {
        await tester.pump(const Duration(milliseconds: 9));
        final double hintOpacity9ms = getOpacity(tester, 'hint');
        expect(hintOpacity9ms, inExclusiveRange(0.0, 1.0));
        await tester.pump(const Duration(milliseconds: 9));
        final double hintOpacity18ms = getOpacity(tester, 'hint');
        expect(hintOpacity18ms, inExclusiveRange(hintOpacity9ms, 1.0));
        await tester.pump(const Duration(milliseconds: 9));
        expect(getOpacity(tester, 'hint'), 1.0);
      }

      // Unfocus to hide the hint.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          decoration: const InputDecoration(labelText: 'label', hintText: 'hint'),
        ),
      );

      // The hint's opacity animates from 1.0 to 0.0.
      // The animation's default duration is 20ms.
      {
        await tester.pump(const Duration(milliseconds: 9));
        final double hintOpacity9ms = getOpacity(tester, 'hint');
        expect(hintOpacity9ms, inExclusiveRange(0.0, 1.0));
        await tester.pump(const Duration(milliseconds: 9));
        final double hintOpacity18ms = getOpacity(tester, 'hint');
        expect(hintOpacity18ms, inExclusiveRange(0.0, hintOpacity9ms));
        await tester.pump(const Duration(milliseconds: 9));
        expect(getOpacity(tester, 'hint'), 0.0);
      }
    });

    testWidgets('InputDecorator custom hint animation duration', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          decoration: const InputDecoration(
            labelText: 'label',
            hintText: 'hint',
            hintFadeDuration: Duration(milliseconds: 120),
          ),
        ),
      );

      // The hint is not visible (opacity 0.0).
      expect(getOpacity(tester, 'hint'), 0.0);

      // Focus to show the hint.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          isFocused: true,
          decoration: const InputDecoration(
            labelText: 'label',
            hintText: 'hint',
            hintFadeDuration: Duration(milliseconds: 120),
          ),
        ),
      );

      // The hint's opacity animates from 0.0 to 1.0.
      // The animation's duration is set to 120ms.
      {
        await tester.pump(const Duration(milliseconds: 50));
        final double hintOpacity50ms = getOpacity(tester, 'hint');
        expect(hintOpacity50ms, inExclusiveRange(0.0, 1.0));
        await tester.pump(const Duration(milliseconds: 50));
        final double hintOpacity100ms = getOpacity(tester, 'hint');
        expect(hintOpacity100ms, inExclusiveRange(hintOpacity50ms, 1.0));
        await tester.pump(const Duration(milliseconds: 50));
        expect(getOpacity(tester, 'hint'), 1.0);
      }

      // Unfocus to hide the hint.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          decoration: const InputDecoration(
            labelText: 'label',
            hintText: 'hint',
            hintFadeDuration: Duration(milliseconds: 120),
          ),
        ),
      );

      // The hint's opacity animates from 1.0 to 0.0.
      // The animation's default duration is 20ms.
      {
        await tester.pump(const Duration(milliseconds: 50));
        final double hintOpacity50ms = getOpacity(tester, 'hint');
        expect(hintOpacity50ms, inExclusiveRange(0.0, 1.0));
        await tester.pump(const Duration(milliseconds: 50));
        final double hintOpacity100ms = getOpacity(tester, 'hint');
        expect(hintOpacity100ms, inExclusiveRange(0.0, hintOpacity50ms));
        await tester.pump(const Duration(milliseconds: 50));
        expect(getOpacity(tester, 'hint'), 0.0);
      }
    });

    testWidgets('InputDecorator custom hint animation duration from theme', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          inputDecorationTheme: const InputDecorationTheme(
            hintFadeDuration: Duration(milliseconds: 120),
          ),
          isEmpty: true,
          decoration: const InputDecoration(labelText: 'label', hintText: 'hint'),
        ),
      );

      // The hint is not visible (opacity 0.0).
      expect(getOpacity(tester, 'hint'), 0.0);

      // Focus to show the hint.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          inputDecorationTheme: const InputDecorationTheme(
            hintFadeDuration: Duration(milliseconds: 120),
          ),
          isEmpty: true,
          isFocused: true,
          decoration: const InputDecoration(labelText: 'label', hintText: 'hint'),
        ),
      );

      // The hint's opacity animates from 0.0 to 1.0.
      // The animation's duration is set to 120ms.
      {
        await tester.pump(const Duration(milliseconds: 50));
        final double hintOpacity50ms = getOpacity(tester, 'hint');
        expect(hintOpacity50ms, inExclusiveRange(0.0, 1.0));
        await tester.pump(const Duration(milliseconds: 50));
        final double hintOpacity100ms = getOpacity(tester, 'hint');
        expect(hintOpacity100ms, inExclusiveRange(hintOpacity50ms, 1.0));
        await tester.pump(const Duration(milliseconds: 50));
        expect(getOpacity(tester, 'hint'), 1.0);
      }

      // Unfocus to hide the hint.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          inputDecorationTheme: const InputDecorationTheme(
            hintFadeDuration: Duration(milliseconds: 120),
          ),
          isEmpty: true,
          decoration: const InputDecoration(labelText: 'label', hintText: 'hint'),
        ),
      );

      // The hint's opacity animates from 1.0 to 0.0.
      // The animation's duration is set to 160ms.
      {
        await tester.pump(const Duration(milliseconds: 50));
        final double hintOpacity50ms = getOpacity(tester, 'hint');
        expect(hintOpacity50ms, inExclusiveRange(0.0, 1.0));
        await tester.pump(const Duration(milliseconds: 50));
        final double hintOpacity100ms = getOpacity(tester, 'hint');
        expect(hintOpacity100ms, inExclusiveRange(0.0, hintOpacity50ms));
        await tester.pump(const Duration(milliseconds: 50));
        expect(getOpacity(tester, 'hint'), 0.0);
      }
    });

    testWidgets('InputDecorator with no input border', (WidgetTester tester) async {
      // Label is visible, hint is not (opacity 0.0).
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          // isFocused: false (default)
          decoration: const InputDecoration(border: InputBorder.none),
        ),
      );
      expect(getBorderWeight(tester), 0.0);
    });

    testWidgets('InputDecorator error/helper/counter layout', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          // isFocused: false (default)
          decoration: const InputDecoration(
            labelText: 'label',
            helperText: 'helper',
            counterText: 'counter',
            filled: true,
          ),
        ),
      );

      // Overall height for this InputDecorator is 76dps. When the label is
      // floating the layout is:
      //
      //   12 - top padding
      //   12 - floating label (font size 16dps * 0.75 = 12)
      //    4 - floating label / input text gap
      //   16 - input text (font size 16dps)
      //   12 - bottom padding
      //    8 - below the border padding
      //   12 - help/error/counter text (font size 12dps)
      //
      // When the label is not floating, it's vertically centered in the space
      // above the subtext:
      //
      //   20 - top padding
      //   16 - label (font size 16dps)
      //   20 - bottom padding (empty input text still appears here)
      //    8 - below the border padding
      //   12 - help/error/counter text (font size 12dps)

      // isEmpty: true, the label is not floating
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 76.0));
      expect(tester.getTopLeft(find.text('text')).dy, 28.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 44.0);
      expect(tester.getTopLeft(find.text('label')).dy, 20.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 36.0);
      expect(getBorderBottom(tester), 56.0);
      expect(getBorderWeight(tester), 1.0);
      expect(tester.getTopLeft(find.text('helper')), const Offset(12.0, 64.0));
      expect(tester.getTopRight(find.text('counter')), const Offset(788.0, 64.0));

      // If errorText is specified then the helperText isn't shown
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: const InputDecoration(
            labelText: 'label',
            errorText: 'error',
            helperText: 'helper',
            counterText: 'counter',
            filled: true,
          ),
        ),
      );
      await tester.pumpAndSettle();

      // isEmpty: false, the label _is_ floating
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 76.0));
      expect(tester.getTopLeft(find.text('text')).dy, 28.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 44.0);
      expect(tester.getTopLeft(find.text('label')).dy, 12.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 24.0);
      expect(getBorderBottom(tester), 56.0);
      expect(getBorderWeight(tester), 1.0);
      expect(tester.getTopLeft(find.text('error')), const Offset(12.0, 64.0));
      expect(tester.getTopRight(find.text('counter')), const Offset(788.0, 64.0));
      expect(find.text('helper'), findsNothing);

      // Overall height for this dense layout InputDecorator is 68dps. When the
      // label is floating the layout is:
      //
      //    8 - top padding
      //   12 - floating label (font size 16dps * 0.75 = 12)
      //    4 - floating label / input text gap
      //   16 - input text (font size 16dps)
      //    8 - bottom padding
      //    8 - below the border padding
      //   12 - help/error/counter text (font size 12dps)
      //
      // When the label is not floating, it's vertically centered in the space
      // above the subtext:
      //
      //   16 - top padding
      //   16 - label (font size 16dps)
      //   16 - bottom padding (empty input text still appears here)
      //    8 - below the border padding
      //   12 - help/error/counter text (font size 12dps)
      // The layout of the error/helper/counter subtext doesn't change for dense layout.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: const InputDecoration(
            isDense: true,
            labelText: 'label',
            errorText: 'error',
            helperText: 'helper',
            counterText: 'counter',
            filled: true,
          ),
        ),
      );
      await tester.pumpAndSettle();

      // isEmpty: false, the label _is_ floating
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 68.0));
      expect(tester.getTopLeft(find.text('text')).dy, 24.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 40.0);
      expect(tester.getTopLeft(find.text('label')).dy, 8.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 20.0);
      expect(getBorderBottom(tester), 48.0);
      expect(getBorderWeight(tester), 1.0);
      expect(tester.getTopLeft(find.text('error')), const Offset(12.0, 56.0));
      expect(tester.getTopRight(find.text('counter')), const Offset(788.0, 56.0));

      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          // isFocused: false (default)
          decoration: const InputDecoration(
            isDense: true,
            labelText: 'label',
            errorText: 'error',
            helperText: 'helper',
            counterText: 'counter',
            filled: true,
          ),
        ),
      );
      await tester.pumpAndSettle();

      // isEmpty: false, the label is not floating
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 68.0));
      expect(tester.getTopLeft(find.text('text')).dy, 24.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 40.0);
      expect(tester.getTopLeft(find.text('label')).dy, 16.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 32.0);
      expect(getBorderBottom(tester), 48.0);
      expect(getBorderWeight(tester), 1.0);
      expect(tester.getTopLeft(find.text('error')), const Offset(12.0, 56.0));
      expect(tester.getTopRight(find.text('counter')), const Offset(788.0, 56.0));
    });

    testWidgets('InputDecoration errorMaxLines', (WidgetTester tester) async {
      const String kError1 = 'e0';
      const String kError2 = 'e0\ne1';
      const String kError3 = 'e0\ne1\ne2';

      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          // isFocused: false (default)
          decoration: const InputDecoration(
            labelText: 'label',
            helperText: 'helper',
            errorText: kError3,
            errorMaxLines: 3,
            filled: true,
          ),
        ),
      );

      // Overall height for this InputDecorator is 100dps:
      //
      //   12 - top padding
      //   12 - floating label (font size 16dps * 0.75 = 12)
      //    4 - floating label / input text gap
      //   16 - input text (font size 16dps)
      //   12 - bottom padding
      //    8 - below the border padding
      //   36 - error text (3 lines, font size 12dps)

      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 100.0));
      expect(tester.getTopLeft(find.text(kError3)), const Offset(12.0, 64.0));
      expect(tester.getBottomLeft(find.text(kError3)), const Offset(12.0, 100.0));

      // Overall height for this InputDecorator is 12 less than the first
      // one, 88dps, because errorText only occupies two lines.

      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          // isFocused: false (default)
          decoration: const InputDecoration(
            labelText: 'label',
            helperText: 'helper',
            errorText: kError2,
            errorMaxLines: 3,
            filled: true,
          ),
        ),
      );

      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 88.0));
      expect(tester.getTopLeft(find.text(kError2)), const Offset(12.0, 64.0));
      expect(tester.getBottomLeft(find.text(kError2)), const Offset(12.0, 88.0));

      // Overall height for this InputDecorator is 24 less than the first
      // one, 88dps, because errorText only occupies one line.

      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          // isFocused: false (default)
          decoration: const InputDecoration(
            labelText: 'label',
            helperText: 'helper',
            errorText: kError1,
            errorMaxLines: 3,
            filled: true,
          ),
        ),
      );

      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 76.0));
      expect(tester.getTopLeft(find.text(kError1)), const Offset(12.0, 64.0));
      expect(tester.getBottomLeft(find.text(kError1)), const Offset(12.0, 76.0));
    });

    testWidgets('InputDecoration helperMaxLines', (WidgetTester tester) async {
      const String kHelper1 = 'e0';
      const String kHelper2 = 'e0\ne1';
      const String kHelper3 = 'e0\ne1\ne2';

      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          // isFocused: false (default)
          decoration: const InputDecoration(
            labelText: 'label',
            helperText: kHelper3,
            helperMaxLines: 3,
            filled: true,
          ),
        ),
      );

      // Overall height for this InputDecorator is 100dps:
      //
      //   12 - top padding
      //   12 - floating label (font size 16dps * 0.75 = 12)
      //    4 - floating label / input text gap
      //   16 - input text (font size 16dps)
      //   12 - bottom padding
      //    8 - below the border padding
      //   36 - helper text (3 lines, font size 12dps)

      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 100.0));
      expect(tester.getTopLeft(find.text(kHelper3)), const Offset(12.0, 64.0));
      expect(tester.getBottomLeft(find.text(kHelper3)), const Offset(12.0, 100.0));

      // Overall height for this InputDecorator is 12 less than the first
      // one, 88dps, because helperText only occupies two lines.

      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          // isFocused: false (default)
          decoration: const InputDecoration(
            labelText: 'label',
            helperText: kHelper3,
            helperMaxLines: 2,
            filled: true,
          ),
        ),
      );

      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 88.0));
      expect(tester.getTopLeft(find.text(kHelper3)), const Offset(12.0, 64.0));
      expect(tester.getBottomLeft(find.text(kHelper3)), const Offset(12.0, 88.0));

      // Overall height for this InputDecorator is 12 less than the first
      // one, 88dps, because helperText only occupies two lines.

      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          // isFocused: false (default)
          decoration: const InputDecoration(
            labelText: 'label',
            helperText: kHelper2,
            helperMaxLines: 3,
            filled: true,
          ),
        ),
      );

      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 88.0));
      expect(tester.getTopLeft(find.text(kHelper2)), const Offset(12.0, 64.0));
      expect(tester.getBottomLeft(find.text(kHelper2)), const Offset(12.0, 88.0));

      // Overall height for this InputDecorator is 24 less than the first
      // one, 88dps, because helperText only occupies one line.

      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          // isFocused: false (default)
          decoration: const InputDecoration(
            labelText: 'label',
            helperText: kHelper1,
            helperMaxLines: 3,
            filled: true,
          ),
        ),
      );

      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 76.0));
      expect(tester.getTopLeft(find.text(kHelper1)), const Offset(12.0, 64.0));
      expect(tester.getBottomLeft(find.text(kHelper1)), const Offset(12.0, 76.0));
    });

    testWidgets('InputDecorator shows helper text', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(decoration: const InputDecoration(helperText: 'helperText')),
      );

      expect(find.text('helperText'), findsOneWidget);
    });

    testWidgets('InputDecorator shows helper widget', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          decoration: const InputDecoration(
            helper: Text('helper', style: TextStyle(fontSize: 20.0)),
          ),
        ),
      );

      expect(find.text('helper'), findsOneWidget);
    });

    testWidgets('InputDecorator throws when helper text and helper widget are provided', (
      WidgetTester tester,
    ) async {
      expect(() {
        buildInputDecoratorM2(
          decoration: InputDecoration(
            helperText: 'helperText',
            helper: const Text('helper', style: TextStyle(fontSize: 20.0)),
          ),
        );
      }, throwsAssertionError);
    });

    testWidgets('InputDecorator shows error text', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(decoration: const InputDecoration(errorText: 'errorText')),
      );

      expect(find.text('errorText'), findsOneWidget);
    });

    testWidgets('InputDecoration shows error border for errorText and error widget', (
      WidgetTester tester,
    ) async {
      const InputBorder errorBorder = OutlineInputBorder(
        borderSide: BorderSide(color: Colors.red, width: 1.5),
      );
      const InputBorder focusedErrorBorder = OutlineInputBorder(
        borderSide: BorderSide(color: Colors.teal, width: 5.0),
      );

      await tester.pumpWidget(
        buildInputDecoratorM2(
          isFocused: true,
          decoration: const InputDecoration(
            errorText: 'error',
            // enabled: true (default)
            errorBorder: errorBorder,
            focusedErrorBorder: focusedErrorBorder,
          ),
        ),
      );
      await tester.pumpAndSettle(); // Border changes are animated.
      expect(getBorder(tester), focusedErrorBorder);

      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isFocused: false (default)
          decoration: const InputDecoration(
            errorText: 'error',
            // enabled: true (default)
            errorBorder: errorBorder,
            focusedErrorBorder: focusedErrorBorder,
          ),
        ),
      );
      await tester.pumpAndSettle(); // Border changes are animated.
      expect(getBorder(tester), errorBorder);

      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isFocused: false (default)
          decoration: const InputDecoration(
            errorText: 'error',
            enabled: false,
            errorBorder: errorBorder,
            focusedErrorBorder: focusedErrorBorder,
          ),
        ),
      );
      await tester.pumpAndSettle(); // Border changes are animated.
      expect(getBorder(tester), errorBorder);

      await tester.pumpWidget(
        buildInputDecoratorM2(
          isFocused: true,
          decoration: const InputDecoration(
            error: Text('error'),
            // enabled: true (default)
            errorBorder: errorBorder,
            focusedErrorBorder: focusedErrorBorder,
          ),
        ),
      );
      await tester.pumpAndSettle(); // Border changes are animated.
      expect(getBorder(tester), focusedErrorBorder);

      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isFocused: false (default)
          decoration: const InputDecoration(
            error: Text('error'),
            // enabled: true (default)
            errorBorder: errorBorder,
            focusedErrorBorder: focusedErrorBorder,
          ),
        ),
      );
      await tester.pumpAndSettle(); // Border changes are animated.
      expect(getBorder(tester), errorBorder);

      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isFocused: false (default)
          decoration: const InputDecoration(
            error: Text('error'),
            enabled: false,
            errorBorder: errorBorder,
            focusedErrorBorder: focusedErrorBorder,
          ),
        ),
      );
      await tester.pumpAndSettle(); // Border changes are animated.
      expect(getBorder(tester), errorBorder);
    });

    testWidgets('InputDecorator shows error widget', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          decoration: const InputDecoration(error: Text('error', style: TextStyle(fontSize: 20.0))),
        ),
      );

      expect(find.text('error'), findsOneWidget);
    });

    testWidgets('InputDecorator throws when error text and error widget are provided', (
      WidgetTester tester,
    ) async {
      expect(() {
        buildInputDecoratorM2(
          decoration: InputDecoration(
            errorText: 'errorText',
            error: const Text('error', style: TextStyle(fontSize: 20.0)),
          ),
        );
      }, throwsAssertionError);
    });

    testWidgets('InputDecorator prefix/suffix texts', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: const InputDecoration(prefixText: 'p', suffixText: 's', filled: true),
        ),
      );

      // Overall height for this InputDecorator is 40dps:
      //   12 - top padding
      //   16 - input text (font size 16dps)
      //   12 - bottom padding
      //
      // The prefix and suffix wrap the input text and are left and right justified
      // respectively. They should have the same height as the input text (16).

      expect(
        tester.getSize(find.byType(InputDecorator)),
        const Size(800.0, kMinInteractiveDimension),
      );
      expect(tester.getSize(find.text('text')).height, 16.0);
      expect(tester.getSize(find.text('p')).height, 16.0);
      expect(tester.getSize(find.text('s')).height, 16.0);
      expect(tester.getTopLeft(find.text('text')).dy, 16.0);
      expect(tester.getTopLeft(find.text('p')).dy, 16.0);
      expect(tester.getTopLeft(find.text('p')).dx, 12.0);
      expect(tester.getTopLeft(find.text('s')).dy, 16.0);
      expect(tester.getTopRight(find.text('s')).dx, 788.0);

      // layout is a row: [p text s]
      expect(tester.getTopLeft(find.text('p')).dx, 12.0);
      expect(
        tester.getTopRight(find.text('p')).dx,
        lessThanOrEqualTo(tester.getTopLeft(find.text('text')).dx),
      );
      expect(
        tester.getTopRight(find.text('text')).dx,
        lessThanOrEqualTo(tester.getTopLeft(find.text('s')).dx),
      );
    });

    testWidgets('InputDecorator icon/prefix/suffix', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: const InputDecoration(
            prefixText: 'p',
            suffixText: 's',
            icon: Icon(Icons.android),
            filled: true,
          ),
        ),
      );

      // Overall height for this InputDecorator is 40dps:
      //   12 - top padding
      //   16 - input text (font size 16dps)
      //   12 - bottom padding

      expect(
        tester.getSize(find.byType(InputDecorator)),
        const Size(800.0, kMinInteractiveDimension),
      );
      expect(tester.getSize(find.text('text')).height, 16.0);
      expect(tester.getSize(find.text('p')).height, 16.0);
      expect(tester.getSize(find.text('s')).height, 16.0);
      expect(tester.getTopLeft(find.text('text')).dy, 16.0);
      expect(tester.getTopLeft(find.text('p')).dy, 16.0);
      expect(tester.getTopLeft(find.text('s')).dy, 16.0);
      expect(tester.getTopRight(find.text('s')).dx, 788.0);
      expect(tester.getSize(find.byType(Icon)).height, 24.0);

      // The 24dps high icon is centered on the 16dps high input line
      expect(tester.getTopLeft(find.byType(Icon)).dy, 12.0);

      // layout is a row: [icon, p text s]
      expect(tester.getTopLeft(find.byType(Icon)).dx, 0.0);
      expect(
        tester.getTopRight(find.byType(Icon)).dx,
        lessThanOrEqualTo(tester.getTopLeft(find.text('p')).dx),
      );
      expect(
        tester.getTopRight(find.text('p')).dx,
        lessThanOrEqualTo(tester.getTopLeft(find.text('text')).dx),
      );
      expect(
        tester.getTopRight(find.text('text')).dx,
        lessThanOrEqualTo(tester.getTopLeft(find.text('s')).dx),
      );
    });

    testWidgets('InputDecorator prefix/suffix widgets', (WidgetTester tester) async {
      const Key pKey = Key('p');
      const Key sKey = Key('s');
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: const InputDecoration(
            prefix: Padding(key: pKey, padding: EdgeInsets.all(4.0), child: Text('p')),
            suffix: Padding(key: sKey, padding: EdgeInsets.all(4.0), child: Text('s')),
            filled: true,
          ),
        ),
      );

      // Overall height for this InputDecorator is 48dps because
      // the prefix and the suffix widget is surrounded with padding:
      //   12 - top padding
      //    4 - top prefix/suffix padding
      //   16 - input text (font size 16dps)
      //    4 - bottom prefix/suffix padding
      //   12 - bottom padding

      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 48.0));
      expect(tester.getSize(find.text('text')).height, 16.0);
      expect(tester.getSize(find.byKey(pKey)).height, 24.0);
      expect(tester.getSize(find.text('p')).height, 16.0);
      expect(tester.getSize(find.byKey(sKey)).height, 24.0);
      expect(tester.getSize(find.text('s')).height, 16.0);
      expect(tester.getTopLeft(find.text('text')).dy, 16.0);
      expect(tester.getTopLeft(find.byKey(pKey)).dy, 12.0);
      expect(tester.getTopLeft(find.text('p')).dy, 16.0);
      expect(tester.getTopLeft(find.byKey(sKey)).dy, 12.0);
      expect(tester.getTopLeft(find.text('s')).dy, 16.0);
      expect(tester.getTopRight(find.byKey(sKey)).dx, 788.0);
      expect(tester.getTopRight(find.text('s')).dx, 784.0);

      // layout is a row: [prefix text suffix]
      expect(tester.getTopLeft(find.byKey(pKey)).dx, 12.0);
      expect(tester.getTopRight(find.byKey(pKey)).dx, tester.getTopLeft(find.text('text')).dx);
      expect(
        tester.getTopRight(find.text('text')).dx,
        lessThanOrEqualTo(tester.getTopRight(find.byKey(sKey)).dx),
      );
    });

    testWidgets('InputDecorator tall prefix', (WidgetTester tester) async {
      const Key pKey = Key('p');
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: const InputDecoration(
            prefix: SizedBox(key: pKey, height: 100, width: 10),
            filled: true,
          ),
          // Set the fontSize so that everything works out to whole numbers.
          child: const Text('text', style: TextStyle(fontFamily: 'FlutterTest', fontSize: 20.0)),
        ),
      );

      // Overall height for this InputDecorator is ~127.2dps because
      // the prefix is 100dps tall, but it aligns with the input's baseline,
      // overlapping the input a bit.
      //   12 - top padding
      //  100 - total height of prefix
      //  -15 - input prefix overlap (distance input top to baseline = 20 * 0.75)
      //   20 - input text (font size 16dps)
      //    0 - bottom prefix/suffix padding
      //   12 - bottom padding

      expect(tester.getSize(find.byType(InputDecorator)).width, 800.0);
      expect(tester.getSize(find.byType(InputDecorator)).height, 129.0);
      expect(tester.getSize(find.text('text')).height, 20.0);
      expect(tester.getSize(find.byKey(pKey)).height, 100.0);
      expect(tester.getTopLeft(find.text('text')).dy, 97); // 12 + 100 - 15
      expect(tester.getTopLeft(find.byKey(pKey)).dy, 12.0);

      // layout is a row: [prefix text suffix]
      expect(tester.getTopLeft(find.byKey(pKey)).dx, 12.0);
      expect(tester.getTopRight(find.byKey(pKey)).dx, tester.getTopLeft(find.text('text')).dx);
    });

    testWidgets('InputDecorator tall prefix with border', (WidgetTester tester) async {
      const Key pKey = Key('p');
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: const InputDecoration(
            border: OutlineInputBorder(),
            prefix: SizedBox(key: pKey, height: 100, width: 10),
            filled: true,
          ),
          // Set the fontSize so that everything works out to whole numbers.
          child: const Text('text', style: TextStyle(fontFamily: 'FlutterTest', fontSize: 20.0)),
        ),
      );

      // Overall height for this InputDecorator is ~127.2dps because
      // the prefix is 100dps tall, but it aligns with the input's baseline,
      // overlapping the input a bit.
      //   24 - top padding
      //  100 - total height of prefix
      //  -15 - input prefix overlap (distance input top to baseline, not exact)
      //   20 - input text (font size 16dps)
      //    0 - bottom prefix/suffix padding
      //   16 - bottom padding
      // When a border is present, the input text and prefix/suffix are centered
      // within the input. Here, that will be content of height 106, centered
      // within an input of height 145. That gives 20 pixels of space on each side
      // of the content, so the prefix is positioned at 19, and the text is at
      // 20+100-15=105.

      expect(tester.getSize(find.byType(InputDecorator)).width, 800.0);
      expect(tester.getSize(find.byType(InputDecorator)).height, 145);
      expect(tester.getSize(find.text('text')).height, 20.0);
      expect(tester.getSize(find.byKey(pKey)).height, 100.0);
      expect(tester.getTopLeft(find.text('text')).dy, 105);
      expect(tester.getTopLeft(find.byKey(pKey)).dy, 20.0);

      // layout is a row: [prefix text suffix]
      expect(tester.getTopLeft(find.byKey(pKey)).dx, 12.0);
      expect(tester.getTopRight(find.byKey(pKey)).dx, tester.getTopLeft(find.text('text')).dx);
    });

    testWidgets('InputDecorator prefixIcon/suffixIcon', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: const InputDecoration(
            prefixIcon: Icon(Icons.pages),
            suffixIcon: Icon(Icons.satellite),
            filled: true,
          ),
        ),
      );

      // Overall height for this InputDecorator is 48dps because the prefix icon's minimum size
      // is 48x48 and the rest of the elements only require 40dps:
      //   12 - top padding
      //   16 - input text (font size 16dps)
      //   12 - bottom padding

      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 48.0));
      expect(tester.getSize(find.text('text')).height, 16.0);
      expect(tester.getSize(find.byIcon(Icons.pages)).height, 48.0);
      expect(tester.getSize(find.byIcon(Icons.satellite)).height, 48.0);
      expect(tester.getTopLeft(find.text('text')).dy, 12.0);
      expect(tester.getTopLeft(find.byIcon(Icons.pages)).dy, 0.0);
      expect(tester.getTopLeft(find.byIcon(Icons.satellite)).dy, 0.0);
      expect(tester.getTopRight(find.byIcon(Icons.satellite)).dx, 800.0);

      // layout is a row: [icon text icon]
      expect(tester.getTopLeft(find.byIcon(Icons.pages)).dx, 0.0);
      expect(
        tester.getTopRight(find.byIcon(Icons.pages)).dx,
        lessThanOrEqualTo(tester.getTopLeft(find.text('text')).dx),
      );
      expect(
        tester.getTopRight(find.text('text')).dx,
        lessThanOrEqualTo(tester.getTopLeft(find.byIcon(Icons.satellite)).dx),
      );
    });

    testWidgets('Material2 - InputDecorator suffixIcon color in error state', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(
            child: TextField(
              decoration: InputDecoration(
                suffixIcon: IconButton(icon: const Icon(Icons.close), onPressed: () {}),
                errorText: 'Error state',
                filled: true,
              ),
            ),
          ),
        ),
      );

      final ThemeData theme = Theme.of(tester.element(find.byType(TextField)));
      expect(getIconStyle(tester, Icons.close)?.color, theme.colorScheme.error);
    });

    testWidgets('InputDecorator prefixIconConstraints/suffixIconConstraints', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: const InputDecoration(
            prefixIcon: Icon(Icons.pages),
            prefixIconConstraints: BoxConstraints(minWidth: 32, minHeight: 32),
            suffixIcon: Icon(Icons.satellite),
            suffixIconConstraints: BoxConstraints(minWidth: 25, minHeight: 25),
            isDense: true, // has to be true to go below 48px height
          ),
        ),
      );

      // Overall height for this InputDecorator is 32px because the prefix icon
      // is now a custom value
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 32.0));
      expect(tester.getSize(find.text('text')).height, 16.0);
      expect(tester.getSize(find.byIcon(Icons.pages)).height, 32.0);
      expect(tester.getSize(find.byIcon(Icons.satellite)).height, 25.0);

      // (InputDecorator height - Text widget height) / 2
      expect(tester.getTopLeft(find.text('text')).dy, (32.0 - 16.0) / 2);
      // prefixIcon should take up the entire height of InputDecorator
      expect(tester.getTopLeft(find.byIcon(Icons.pages)).dy, 0.0);
      // (InputDecorator height - suffixIcon height) / 2
      expect(tester.getTopLeft(find.byIcon(Icons.satellite)).dy, (32.0 - 25.0) / 2);
      expect(tester.getTopRight(find.byIcon(Icons.satellite)).dx, 800.0);
    });

    testWidgets('prefix/suffix icons are centered when smaller than 48 by 48', (
      WidgetTester tester,
    ) async {
      const Key prefixKey = Key('prefix');
      await tester.pumpWidget(
        buildInputDecoratorM2(
          decoration: const InputDecoration(
            prefixIcon: Padding(
              padding: EdgeInsets.all(16.0),
              child: SizedBox(width: 8.0, height: 8.0, key: prefixKey),
            ),
            filled: true,
          ),
        ),
      );

      // Overall height for this InputDecorator is 48dps because the prefix icon's minimum size
      // is 48x48 and the rest of the elements only require 40dps:
      //   12 - top padding
      //   16 - input text (font size 16dps)
      //   12 - bottom padding

      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 48.0));
      expect(tester.getSize(find.byKey(prefixKey)).height, 16.0);
      expect(tester.getTopLeft(find.byKey(prefixKey)).dy, 16.0);
    });

    testWidgets('InputDecorator respects reduced theme visualDensity', (WidgetTester tester) async {
      // Label is visible, hint is not (opacity 0.0).
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          visualDensity: VisualDensity.compact,
          decoration: const InputDecoration(labelText: 'label', hintText: 'hint'),
        ),
      );

      // The label is not floating so it's vertically centered.
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 48.0));
      expect(tester.getTopLeft(find.text('text')).dy, 24.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 40.0);
      expect(tester.getTopLeft(find.text('label')).dy, 16.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 32.0);
      expect(getOpacity(tester, 'hint'), 0.0);
      expect(getBorderBottom(tester), 48.0);
      expect(getBorderWeight(tester), 1.0);

      // Label moves upwards, hint is visible (opacity 1.0).
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          isFocused: true,
          visualDensity: VisualDensity.compact,
          decoration: const InputDecoration(labelText: 'label', hintText: 'hint'),
        ),
      );

      // The hint's opacity animates from 0.0 to 1.0.
      // The animation's default duration is 20ms.
      {
        await tester.pump(const Duration(milliseconds: 9));
        final double hintOpacity9ms = getOpacity(tester, 'hint');
        expect(hintOpacity9ms, inExclusiveRange(0.0, 1.0));
        await tester.pump(const Duration(milliseconds: 9));
        final double hintOpacity18ms = getOpacity(tester, 'hint');
        expect(hintOpacity18ms, inExclusiveRange(hintOpacity9ms, 1.0));
      }

      await tester.pumpAndSettle();
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 48.0));
      expect(tester.getTopLeft(find.text('text')).dy, 24.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 40.0);
      expect(tester.getTopLeft(find.text('label')).dy, 8.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 20.0);
      expect(tester.getTopLeft(find.text('hint')).dy, 24.0);
      expect(tester.getBottomLeft(find.text('hint')).dy, 40.0);
      expect(getOpacity(tester, 'hint'), 1.0);
      expect(getBorderBottom(tester), 48.0);
      expect(getBorderWeight(tester), 2.0);

      await tester.pumpWidget(
        buildInputDecoratorM2(
          isFocused: true,
          visualDensity: VisualDensity.compact,
          decoration: const InputDecoration(labelText: 'label', hintText: 'hint'),
        ),
      );

      // The hint's opacity animates from 1.0 to 0.0.
      // The animation's default duration is 20ms.
      {
        await tester.pump(const Duration(milliseconds: 9));
        final double hintOpacity9ms = getOpacity(tester, 'hint');
        expect(hintOpacity9ms, inExclusiveRange(0.0, 1.0));
        await tester.pump(const Duration(milliseconds: 9));
        final double hintOpacity18ms = getOpacity(tester, 'hint');
        expect(hintOpacity18ms, inExclusiveRange(0.0, hintOpacity9ms));
      }

      await tester.pumpAndSettle();
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 48.0));
      expect(tester.getTopLeft(find.text('text')).dy, 24.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 40.0);
      expect(tester.getTopLeft(find.text('label')).dy, 8.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 20.0);
      expect(tester.getTopLeft(find.text('hint')).dy, 24.0);
      expect(tester.getBottomLeft(find.text('hint')).dy, 40.0);
      expect(getOpacity(tester, 'hint'), 0.0);
      expect(getBorderBottom(tester), 48.0);
      expect(getBorderWeight(tester), 2.0);
    });

    testWidgets('InputDecorator respects increased theme visualDensity', (
      WidgetTester tester,
    ) async {
      // Label is visible, hint is not (opacity 0.0).
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          visualDensity: const VisualDensity(horizontal: 2.0, vertical: 2.0),
          decoration: const InputDecoration(labelText: 'label', hintText: 'hint'),
        ),
      );

      // The label is not floating so it's vertically centered.
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 64.0));
      expect(tester.getTopLeft(find.text('text')).dy, 32.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 48.0);
      expect(tester.getTopLeft(find.text('label')).dy, 24.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 40.0);
      expect(getOpacity(tester, 'hint'), 0.0);
      expect(getBorderBottom(tester), 64.0);
      expect(getBorderWeight(tester), 1.0);

      // Label moves upwards, hint is visible (opacity 1.0).
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          isFocused: true,
          visualDensity: const VisualDensity(horizontal: 2.0, vertical: 2.0),
          decoration: const InputDecoration(labelText: 'label', hintText: 'hint'),
        ),
      );

      // The hint's opacity animates from 0.0 to 1.0.
      // The animation's default duration is 20ms.
      {
        await tester.pump(const Duration(milliseconds: 9));
        final double hintOpacity9ms = getOpacity(tester, 'hint');
        expect(hintOpacity9ms, inExclusiveRange(0.0, 1.0));
        await tester.pump(const Duration(milliseconds: 9));
        final double hintOpacity18ms = getOpacity(tester, 'hint');
        expect(hintOpacity18ms, inExclusiveRange(hintOpacity9ms, 1.0));
      }

      await tester.pumpAndSettle();
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 64.0));
      expect(tester.getTopLeft(find.text('text')).dy, 32.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 48.0);
      expect(tester.getTopLeft(find.text('label')).dy, 16.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 28.0);
      expect(tester.getTopLeft(find.text('hint')).dy, 32.0);
      expect(tester.getBottomLeft(find.text('hint')).dy, 48.0);
      expect(getOpacity(tester, 'hint'), 1.0);
      expect(getBorderBottom(tester), 64.0);
      expect(getBorderWeight(tester), 2.0);

      await tester.pumpWidget(
        buildInputDecoratorM2(
          isFocused: true,
          visualDensity: const VisualDensity(horizontal: 2.0, vertical: 2.0),
          decoration: const InputDecoration(labelText: 'label', hintText: 'hint'),
        ),
      );

      // The hint's opacity animates from 1.0 to 0.0.
      // The animation's default duration is 20ms.
      {
        await tester.pump(const Duration(milliseconds: 9));
        final double hintOpacity9ms = getOpacity(tester, 'hint');
        expect(hintOpacity9ms, inExclusiveRange(0.0, 1.0));
        await tester.pump(const Duration(milliseconds: 9));
        final double hintOpacity18ms = getOpacity(tester, 'hint');
        expect(hintOpacity18ms, inExclusiveRange(0.0, hintOpacity9ms));
      }

      await tester.pumpAndSettle();
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 64.0));
      expect(tester.getTopLeft(find.text('text')).dy, 32.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 48.0);
      expect(tester.getTopLeft(find.text('label')).dy, 16.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 28.0);
      expect(tester.getTopLeft(find.text('hint')).dy, 32.0);
      expect(tester.getBottomLeft(find.text('hint')).dy, 48.0);
      expect(getOpacity(tester, 'hint'), 0.0);
      expect(getBorderBottom(tester), 64.0);
      expect(getBorderWeight(tester), 2.0);
    });

    testWidgets('prefix/suffix icons increase height of decoration when larger than 48 by 48', (
      WidgetTester tester,
    ) async {
      const Key prefixKey = Key('prefix');
      await tester.pumpWidget(
        buildInputDecoratorM2(
          decoration: const InputDecoration(
            prefixIcon: SizedBox(width: 100.0, height: 100.0, key: prefixKey),
            filled: true,
          ),
        ),
      );

      // Overall height for this InputDecorator is 100dps because the prefix icon's size
      // is 100x100 and the rest of the elements only require 40dps:
      //   12 - top padding
      //   16 - input text (font size 16dps)
      //   12 - bottom padding

      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 100.0));
      expect(tester.getSize(find.byKey(prefixKey)).height, 100.0);
      expect(tester.getTopLeft(find.byKey(prefixKey)).dy, 0.0);
    });

    group('constraints', () {
      testWidgets('No InputDecorator constraints', (WidgetTester tester) async {
        await tester.pumpWidget(buildInputDecoratorM2());

        // Should fill the screen width and be default height
        expect(tester.getSize(find.byType(InputDecorator)), const Size(800, 48));
      });

      testWidgets('InputDecoratorThemeData constraints', (WidgetTester tester) async {
        await tester.pumpWidget(
          buildInputDecoratorM2(
            theme: ThemeData(
              inputDecorationTheme: const InputDecorationTheme(
                constraints: BoxConstraints(maxWidth: 300, maxHeight: 40),
              ),
            ),
          ),
        );

        // Theme settings should make it 300x40 pixels
        expect(tester.getSize(find.byType(InputDecorator)), const Size(300, 40));
      });

      testWidgets('InputDecorator constraints', (WidgetTester tester) async {
        await tester.pumpWidget(
          buildInputDecoratorM2(
            theme: ThemeData(
              inputDecorationTheme: const InputDecorationTheme(
                constraints: BoxConstraints(maxWidth: 300, maxHeight: 40),
              ),
            ),
            decoration: const InputDecoration(
              constraints: BoxConstraints(maxWidth: 200, maxHeight: 32),
            ),
          ),
        );

        // InputDecoration.constraints should override the theme. It should be
        // only 200x32 pixels
        expect(tester.getSize(find.byType(InputDecorator)), const Size(200, 32));
      });
    });

    group('textAlignVertical position', () {
      group('simple case', () {
        testWidgets('align top (default)', (WidgetTester tester) async {
          const String text = 'text';
          await tester.pumpWidget(
            buildInputDecoratorM2(
              // isEmpty: false (default)
              // isFocused: false (default)
              expands: true, // so we have a tall input where align can vary
              decoration: const InputDecoration(filled: true),
              textAlignVertical: TextAlignVertical.top, // default when no border
              // Set the fontSize so that everything works out to whole numbers.
              child: const Text(text, style: TextStyle(fontFamily: 'FlutterTest', fontSize: 20.0)),
            ),
          );

          // Same as the default case above.
          expect(tester.getTopLeft(find.text(text)).dy, 12.0);
        });

        testWidgets('align center', (WidgetTester tester) async {
          const String text = 'text';
          await tester.pumpWidget(
            buildInputDecoratorM2(
              // isEmpty: false (default)
              // isFocused: false (default)
              expands: true,
              decoration: const InputDecoration(filled: true),
              textAlignVertical: TextAlignVertical.center,
              // Set the fontSize so that everything works out to whole numbers.
              child: const Text(text, style: TextStyle(fontFamily: 'FlutterTest', fontSize: 20.0)),
            ),
          );

          // Below the top aligned case.
          expect(tester.getTopLeft(find.text(text)).dy, 290.0);
        });

        testWidgets('align bottom', (WidgetTester tester) async {
          const String text = 'text';
          await tester.pumpWidget(
            buildInputDecoratorM2(
              // isEmpty: false (default)
              // isFocused: false (default)
              expands: true,
              decoration: const InputDecoration(filled: true),
              textAlignVertical: TextAlignVertical.bottom,
              // Set the fontSize so that everything works out to whole numbers.
              child: const Text(text, style: TextStyle(fontFamily: 'FlutterTest', fontSize: 20.0)),
            ),
          );

          // Below the center aligned case.
          expect(tester.getTopLeft(find.text(text)).dy, 568.0);
        });

        testWidgets('align as a double', (WidgetTester tester) async {
          const String text = 'text';
          await tester.pumpWidget(
            buildInputDecoratorM2(
              // isEmpty: false (default)
              // isFocused: false (default)
              expands: true,
              decoration: const InputDecoration(filled: true),
              textAlignVertical: const TextAlignVertical(y: 0.75),
              // Set the fontSize so that everything works out to whole numbers.
              child: const Text(text, style: TextStyle(fontFamily: 'FlutterTest', fontSize: 20.0)),
            ),
          );

          // In between the center and bottom aligned cases.
          expect(tester.getTopLeft(find.text(text)).dy, 498.5);
        });

        testWidgets('works with density and content padding', (WidgetTester tester) async {
          const Key key = Key('child');
          const Key containerKey = Key('container');
          const double totalHeight = 100.0;
          const double childHeight = 20.0;
          const VisualDensity visualDensity = VisualDensity(vertical: VisualDensity.maximumDensity);
          const EdgeInsets contentPadding = EdgeInsets.only(top: 6, bottom: 14);

          await tester.pumpWidget(
            Center(
              child: SizedBox(
                key: containerKey,
                height: totalHeight,
                child: buildInputDecoratorM2(
                  // isEmpty: false (default)
                  // isFocused: false (default)
                  expands: true,
                  decoration: const InputDecoration(
                    border: InputBorder.none,
                    contentPadding: contentPadding,
                  ),
                  textAlignVertical: TextAlignVertical.center,
                  visualDensity: visualDensity,
                  child: const SizedBox(key: key, height: childHeight),
                ),
              ),
            ),
          );

          // Vertical components: contentPadding.vertical, densityOffset.y, child
          final double childVerticalSpaceAffordance =
              totalHeight - visualDensity.baseSizeAdjustment.dy - contentPadding.vertical;

          // TextAlignVertical.center is specified so `child` needs to be centered
          // in the available space.
          final double childMargin = (childVerticalSpaceAffordance - childHeight) / 2;
          final double childTop =
              visualDensity.baseSizeAdjustment.dy / 2.0 + contentPadding.top + childMargin;

          expect(
            tester.getTopLeft(find.byKey(key)).dy,
            tester.getTopLeft(find.byKey(containerKey)).dy + childTop,
          );
        });
      });

      group('outline border', () {
        testWidgets('align top', (WidgetTester tester) async {
          const String text = 'text';
          await tester.pumpWidget(
            buildInputDecoratorM2(
              // isEmpty: false (default)
              // isFocused: false (default)
              expands: true, // so we have a tall input where align can vary
              decoration: const InputDecoration(filled: true, border: OutlineInputBorder()),
              textAlignVertical: TextAlignVertical.top,
              // Set the fontSize so that everything works out to whole numbers.
              child: const Text(text, style: TextStyle(fontFamily: 'FlutterTest', fontSize: 20.0)),
            ),
          );

          // Similar to the case without a border, but with a little extra room at
          // the top to make room for the border.
          expect(tester.getTopLeft(find.text(text)).dy, 24.0);
        });

        testWidgets('align center (default)', (WidgetTester tester) async {
          const String text = 'text';
          await tester.pumpWidget(
            buildInputDecoratorM2(
              // isEmpty: false (default)
              // isFocused: false (default)
              expands: true,
              decoration: const InputDecoration(filled: true, border: OutlineInputBorder()),
              textAlignVertical: TextAlignVertical.center, // default when border
              // Set the fontSize so that everything works out to whole numbers.
              child: const Text(text, style: TextStyle(fontFamily: 'FlutterTest', fontSize: 20.0)),
            ),
          );

          // Below the top aligned case.
          expect(tester.getTopLeft(find.text(text)).dy, 290.0);
        });

        testWidgets('align bottom', (WidgetTester tester) async {
          const String text = 'text';
          await tester.pumpWidget(
            buildInputDecoratorM2(
              // isEmpty: false (default)
              // isFocused: false (default)
              expands: true,
              decoration: const InputDecoration(filled: true, border: OutlineInputBorder()),
              textAlignVertical: TextAlignVertical.bottom,
              // Set the fontSize so that everything works out to whole numbers.
              child: const Text(text, style: TextStyle(fontFamily: 'FlutterTest', fontSize: 20.0)),
            ),
          );

          // Below the center aligned case.
          expect(tester.getTopLeft(find.text(text)).dy, 564.0);
        });
      });

      group('prefix', () {
        testWidgets('InputDecorator tall prefix align top', (WidgetTester tester) async {
          const Key pKey = Key('p');
          const String text = 'text';
          await tester.pumpWidget(
            buildInputDecoratorM2(
              // isEmpty: false (default)
              // isFocused: false (default)
              decoration: const InputDecoration(
                prefix: SizedBox(key: pKey, height: 100, width: 10),
                filled: true,
              ),
              textAlignVertical: TextAlignVertical.top, // default when no border
              // Set the fontSize so that everything works out to whole numbers.
              child: const Text(text, style: TextStyle(fontFamily: 'FlutterTest', fontSize: 20.0)),
            ),
          );

          // Same as the default case above.
          expect(tester.getTopLeft(find.text(text)).dy, 97.0);
          expect(tester.getTopLeft(find.byKey(pKey)).dy, 12.0);
        });

        testWidgets('InputDecorator tall prefix align center', (WidgetTester tester) async {
          const Key pKey = Key('p');
          const String text = 'text';
          await tester.pumpWidget(
            buildInputDecoratorM2(
              // isEmpty: false (default)
              // isFocused: false (default)
              decoration: const InputDecoration(
                prefix: SizedBox(key: pKey, height: 100, width: 10),
                filled: true,
              ),
              textAlignVertical: TextAlignVertical.center,
              // Set the fontSize so that everything works out to whole numbers.
              child: const Text(text, style: TextStyle(fontFamily: 'FlutterTest', fontSize: 20.0)),
            ),
          );

          // Same as the default case above.
          expect(tester.getTopLeft(find.text(text)).dy, 97.0);
          expect(tester.getTopLeft(find.byKey(pKey)).dy, 12.0);
        });

        testWidgets('InputDecorator tall prefix align bottom', (WidgetTester tester) async {
          const Key pKey = Key('p');
          const String text = 'text';
          await tester.pumpWidget(
            buildInputDecoratorM2(
              // isEmpty: false (default)
              // isFocused: false (default)
              decoration: const InputDecoration(
                prefix: SizedBox(key: pKey, height: 100, width: 10),
                filled: true,
              ),
              textAlignVertical: TextAlignVertical.bottom,
              // Set the fontSize so that everything works out to whole numbers.
              child: const Text(text, style: TextStyle(fontFamily: 'FlutterTest', fontSize: 20.0)),
            ),
          );

          // Top of the input + 100 prefix height - overlap
          expect(tester.getTopLeft(find.text(text)).dy, 97.0);
          expect(tester.getTopLeft(find.byKey(pKey)).dy, 12.0);
        });
      });

      group('outline border and prefix', () {
        testWidgets('InputDecorator tall prefix align center', (WidgetTester tester) async {
          const Key pKey = Key('p');
          const String text = 'text';
          await tester.pumpWidget(
            buildInputDecoratorM2(
              // isEmpty: false (default)
              // isFocused: false (default)
              expands: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                prefix: SizedBox(key: pKey, height: 100, width: 10),
                filled: true,
              ),
              textAlignVertical: TextAlignVertical.center, // default when border
              // Set the fontSize so that everything works out to whole numbers.
              child: const Text(text, style: TextStyle(fontFamily: 'FlutterTest', fontSize: 20.0)),
            ),
          );

          // In the middle of the expanded InputDecorator.
          expect(tester.getTopLeft(find.text(text)).dy, 332.5);
          expect(tester.getTopLeft(find.byKey(pKey)).dy, 247.5);
        });

        testWidgets('InputDecorator tall prefix with border align top', (
          WidgetTester tester,
        ) async {
          const Key pKey = Key('p');
          const String text = 'text';
          await tester.pumpWidget(
            buildInputDecoratorM2(
              // isEmpty: false (default)
              // isFocused: false (default)
              expands: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                prefix: SizedBox(key: pKey, height: 100, width: 10),
                filled: true,
              ),
              textAlignVertical: TextAlignVertical.top,
              // Set the fontSize so that everything works out to whole numbers.
              child: const Text(text, style: TextStyle(fontFamily: 'FlutterTest', fontSize: 20.0)),
            ),
          );

          // Above the center example.
          expect(tester.getTopLeft(find.text(text)).dy, 109.0);
          // The prefix is positioned at the top of the input, so this value is
          // the same as the top aligned test without a prefix.
          expect(tester.getTopLeft(find.byKey(pKey)).dy, 24.0);
        });

        testWidgets('InputDecorator tall prefix with border align bottom', (
          WidgetTester tester,
        ) async {
          const Key pKey = Key('p');
          const String text = 'text';
          await tester.pumpWidget(
            buildInputDecoratorM2(
              // isEmpty: false (default)
              // isFocused: false (default)
              expands: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                prefix: SizedBox(key: pKey, height: 100, width: 10),
                filled: true,
              ),
              textAlignVertical: TextAlignVertical.bottom,
              // Set the fontSize so that everything works out to whole numbers.
              child: const Text(text, style: TextStyle(fontFamily: 'FlutterTest', fontSize: 20.0)),
            ),
          );

          // Below the center example.
          expect(tester.getTopLeft(find.text(text)).dy, 564.0);
          expect(tester.getTopLeft(find.byKey(pKey)).dy, 479.0);
        });

        testWidgets('InputDecorator tall prefix with border align double', (
          WidgetTester tester,
        ) async {
          const Key pKey = Key('p');
          const String text = 'text';
          await tester.pumpWidget(
            buildInputDecoratorM2(
              // isEmpty: false (default)
              // isFocused: false (default)
              expands: true,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                prefix: SizedBox(key: pKey, height: 100, width: 10),
                filled: true,
              ),
              textAlignVertical: const TextAlignVertical(y: 0.1),
              // Set the fontSize so that everything works out to whole numbers.
              child: const Text(text, style: TextStyle(fontFamily: 'FlutterTest', fontSize: 20.0)),
            ),
          );

          // Between the top and center examples.
          expect(tester.getTopLeft(find.text(text)).dy, 355.65);
          expect(tester.getTopLeft(find.byKey(pKey)).dy, 270.65);
        });
      });

      group('label', () {
        testWidgets('align top (default)', (WidgetTester tester) async {
          const String text = 'text';
          await tester.pumpWidget(
            buildInputDecoratorM2(
              // isEmpty: false (default)
              // isFocused: false (default)
              expands: true, // so we have a tall input where align can vary
              decoration: const InputDecoration(labelText: 'label', filled: true),
              textAlignVertical: TextAlignVertical.top, // default
              // Set the fontSize so that everything works out to whole numbers.
              child: const Text(text, style: TextStyle(fontFamily: 'FlutterTest', fontSize: 20.0)),
            ),
          );

          // The label causes the text to start slightly lower than it would
          // otherwise.
          expect(tester.getTopLeft(find.text(text)).dy, 28.0);
        });

        testWidgets('align center', (WidgetTester tester) async {
          const String text = 'text';
          await tester.pumpWidget(
            buildInputDecoratorM2(
              // isEmpty: false (default)
              // isFocused: false (default)
              expands: true, // so we have a tall input where align can vary
              decoration: const InputDecoration(labelText: 'label', filled: true),
              textAlignVertical: TextAlignVertical.center,
              // Set the fontSize so that everything works out to whole numbers.
              child: const Text(text, style: TextStyle(fontFamily: 'FlutterTest', fontSize: 20.0)),
            ),
          );

          // The label reduces the amount of space available for text, so the
          // center is slightly lower.
          expect(tester.getTopLeft(find.text(text)).dy, 298.0);
        });

        testWidgets('align bottom', (WidgetTester tester) async {
          const String text = 'text';
          await tester.pumpWidget(
            buildInputDecoratorM2(
              // isEmpty: false (default)
              // isFocused: false (default)
              expands: true, // so we have a tall input where align can vary
              decoration: const InputDecoration(labelText: 'label', filled: true),
              textAlignVertical: TextAlignVertical.bottom,
              // Set the fontSize so that everything works out to whole numbers.
              child: const Text(text, style: TextStyle(fontFamily: 'FlutterTest', fontSize: 20.0)),
            ),
          );

          // The label reduces the amount of space available for text, but the
          // bottom line is still in the same place.
          expect(tester.getTopLeft(find.text(text)).dy, 568.0);
        });
      });
    });

    group('OutlineInputBorder', () {
      group('default alignment', () {
        testWidgets('Centers when border', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecoratorM2(decoration: const InputDecoration(border: OutlineInputBorder())),
          );

          expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
          expect(tester.getTopLeft(find.text('text')).dy, 20.0);
          expect(tester.getBottomLeft(find.text('text')).dy, 36.0);
          expect(getBorderBottom(tester), 56.0);
          expect(getBorderWeight(tester), 1.0);
        });

        testWidgets('Centers when border and label', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecoratorM2(
              decoration: const InputDecoration(labelText: 'label', border: OutlineInputBorder()),
            ),
          );

          expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
          expect(tester.getTopLeft(find.text('text')).dy, 20.0);
          expect(tester.getBottomLeft(find.text('text')).dy, 36.0);
          expect(getBorderBottom(tester), 56.0);
          expect(getBorderWeight(tester), 1.0);
        });

        testWidgets('Centers when border and contentPadding', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecoratorM2(
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                contentPadding: EdgeInsets.fromLTRB(12.0, 14.0, 8.0, 14.0),
              ),
            ),
          );

          expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 48.0));
          expect(tester.getTopLeft(find.text('text')).dy, 16.0);
          expect(tester.getBottomLeft(find.text('text')).dy, 32.0);
          expect(getBorderBottom(tester), 48.0);
          expect(getBorderWeight(tester), 1.0);
        });

        testWidgets('Centers when border and contentPadding and label', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecoratorM2(
              decoration: const InputDecoration(
                labelText: 'label',
                border: OutlineInputBorder(),
                contentPadding: EdgeInsets.fromLTRB(12.0, 14.0, 8.0, 14.0),
              ),
            ),
          );
          expect(
            tester.getSize(find.byType(InputDecorator)),
            const Size(800.0, kMinInteractiveDimension),
          );
          expect(tester.getTopLeft(find.text('text')).dy, 16.0);
          expect(tester.getBottomLeft(find.text('text')).dy, 32.0);
          expect(getBorderBottom(tester), 48.0);
          expect(getBorderWeight(tester), 1.0);
        });

        testWidgets('Centers when border and lopsided contentPadding and label', (
          WidgetTester tester,
        ) async {
          await tester.pumpWidget(
            buildInputDecoratorM2(
              decoration: const InputDecoration(
                labelText: 'label',
                border: OutlineInputBorder(),
                contentPadding: EdgeInsets.fromLTRB(12.0, 104.0, 8.0, 0.0),
              ),
            ),
          );

          expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 120.0));
          expect(tester.getTopLeft(find.text('text')).dy, 52.0);
          expect(tester.getBottomLeft(find.text('text')).dy, 68.0);
          expect(getBorderBottom(tester), 120.0);
          expect(getBorderWeight(tester), 1.0);
        });

        testWidgets('Label aligns horizontally with text', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecoratorM2(
              decoration: const InputDecoration(
                prefixIcon: Icon(Icons.ac_unit),
                labelText: 'label',
                border: OutlineInputBorder(),
              ),
              isFocused: true,
            ),
          );

          expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
          expect(tester.getTopLeft(find.text('label')).dx, 48.0);
          expect(tester.getBottomLeft(find.text('text')).dx, 48.0);
          expect(getBorderWeight(tester), 2.0);
        });

        testWidgets(
          'Floating label for filled input decoration is horizontally aligned with text',
          (WidgetTester tester) async {
            // Regression test added in https://github.com/flutter/flutter/pull/115540.
            await tester.pumpWidget(
              buildInputDecoratorM2(
                decoration: const InputDecoration(
                  prefixIcon: Icon(Icons.ac_unit),
                  labelText: 'label',
                  filled: true,
                ),
                isFocused: true,
              ),
            );

            expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
            expect(tester.getTopLeft(find.text('label')).dx, 48.0);
            expect(tester.getBottomLeft(find.text('text')).dx, 48.0);
            expect(getBorderWeight(tester), 2.0);
          },
        );
      });

      group('3 point interpolation alignment', () {
        testWidgets('top align includes padding', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecoratorM2(
              expands: true,
              textAlignVertical: TextAlignVertical.top,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                contentPadding: EdgeInsets.fromLTRB(12.0, 24.0, 8.0, 2.0),
              ),
            ),
          );

          expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 600.0));
          // Aligned to the top including the 24px padding.
          expect(tester.getTopLeft(find.text('text')).dy, 24.0);
          expect(tester.getBottomLeft(find.text('text')).dy, 40.0);
          expect(getBorderBottom(tester), 600.0);
          expect(getBorderWeight(tester), 1.0);
        });

        testWidgets('center align ignores padding', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecoratorM2(
              expands: true,
              textAlignVertical: TextAlignVertical.center,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                contentPadding: EdgeInsets.fromLTRB(12.0, 24.0, 8.0, 2.0),
              ),
            ),
          );

          expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 600.0));
          // Baseline is on the center of the 600px high input.
          expect(tester.getTopLeft(find.text('text')).dy, 292.0);
          expect(tester.getBottomLeft(find.text('text')).dy, 308.0);
          expect(getBorderBottom(tester), 600.0);
          expect(getBorderWeight(tester), 1.0);
        });

        testWidgets('bottom align includes padding', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecoratorM2(
              expands: true,
              textAlignVertical: TextAlignVertical.bottom,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                contentPadding: EdgeInsets.fromLTRB(12.0, 24.0, 8.0, 2.0),
              ),
            ),
          );

          expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 600.0));
          // Includes bottom padding of 2px.
          expect(tester.getTopLeft(find.text('text')).dy, 582.0);
          expect(tester.getBottomLeft(find.text('text')).dy, 598.0);
          expect(getBorderBottom(tester), 600.0);
          expect(getBorderWeight(tester), 1.0);
        });

        testWidgets('padding exceeds middle keeps top at middle', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecoratorM2(
              expands: true,
              textAlignVertical: TextAlignVertical.top,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                contentPadding: EdgeInsets.fromLTRB(12.0, 504.0, 8.0, 0.0),
              ),
            ),
          );

          expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 600.0));
          // Same position as the center example above.
          expect(tester.getTopLeft(find.text('text')).dy, 292.0);
          expect(tester.getBottomLeft(find.text('text')).dy, 308.0);
          expect(getBorderBottom(tester), 600.0);
          expect(getBorderWeight(tester), 1.0);
        });
      });
    });

    testWidgets('counter text has correct right margin - LTR, not dense', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: const InputDecoration(counterText: 'test', filled: true),
        ),
      );

      // Margin for text decoration is 12 when filled
      // (dx) - 12 = (text offset)x.
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 68.0));
      final double dx = tester.getRect(find.byType(InputDecorator)).right;
      expect(tester.getRect(find.text('test')).right, dx - 12.0);
    });

    testWidgets('counter text has correct right margin - RTL, not dense', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          textDirection: TextDirection.rtl,
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: const InputDecoration(counterText: 'test', filled: true),
        ),
      );

      // Margin for text decoration is 12 when filled and top left offset is (0, 0)
      // 0 + 12 = 12.
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 68.0));
      expect(tester.getRect(find.text('test')).left, 12.0);
    });

    testWidgets('counter text has correct right margin - LTR, dense', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: const InputDecoration(counterText: 'test', filled: true, isDense: true),
        ),
      );

      // Margin for text decoration is 12 when filled
      // (dx) - 12 = (text offset)x.
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 52.0));
      final double dx = tester.getRect(find.byType(InputDecorator)).right;
      expect(tester.getRect(find.text('test')).right, dx - 12.0);
    });

    testWidgets('counter text has correct right margin - RTL, dense', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          textDirection: TextDirection.rtl,
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: const InputDecoration(counterText: 'test', filled: true, isDense: true),
        ),
      );

      // Margin for text decoration is 12 when filled and top left offset is (0, 0)
      // 0 + 12 = 12.
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 52.0));
      expect(tester.getRect(find.text('test')).left, 12.0);
    });

    testWidgets('InputDecorator error/helper/counter RTL layout', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          textDirection: TextDirection.rtl,
          decoration: const InputDecoration(
            labelText: 'label',
            helperText: 'helper',
            counterText: 'counter',
            filled: true,
          ),
        ),
      );

      // Overall height for this InputDecorator is 76dps:
      //   12 - top padding
      //   12 - floating label (font size 16dps * 0.75 = 12)
      //    4 - floating label / input text gap
      //   16 - input text (font size 16dps)
      //   12 - bottom padding
      //    8 - below the border padding
      //   12 - [counter helper/error] (font size 12dps)

      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 76.0));
      expect(tester.getTopLeft(find.text('text')).dy, 28.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 44.0);
      expect(tester.getTopLeft(find.text('label')).dy, 12.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 24.0);
      expect(getBorderBottom(tester), 56.0);
      expect(getBorderWeight(tester), 1.0);
      expect(tester.getTopLeft(find.text('counter')), const Offset(12.0, 64.0));
      expect(tester.getTopRight(find.text('helper')), const Offset(788.0, 64.0));

      // If both error and helper are specified, show the error
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          textDirection: TextDirection.rtl,
          decoration: const InputDecoration(
            labelText: 'label',
            helperText: 'helper',
            errorText: 'error',
            counterText: 'counter',
            filled: true,
          ),
        ),
      );
      await tester.pumpAndSettle();
      expect(tester.getTopLeft(find.text('counter')), const Offset(12.0, 64.0));
      expect(tester.getTopRight(find.text('error')), const Offset(788.0, 64.0));
      expect(find.text('helper'), findsNothing);
    });

    testWidgets('InputDecorator prefix/suffix RTL', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          textDirection: TextDirection.rtl,
          decoration: const InputDecoration(prefixText: 'p', suffixText: 's', filled: true),
        ),
      );

      // Overall height for this InputDecorator is 40dps:
      //   12 - top padding
      //   16 - input text (font size 16dps)
      //   12 - bottom padding

      expect(
        tester.getSize(find.byType(InputDecorator)),
        const Size(800.0, kMinInteractiveDimension),
      ); // 40 bumped up to minimum.
      expect(tester.getSize(find.text('text')).height, 16.0);
      expect(tester.getSize(find.text('p')).height, 16.0);
      expect(tester.getSize(find.text('s')).height, 16.0);
      expect(tester.getTopLeft(find.text('text')).dy, 16.0);
      expect(tester.getTopLeft(find.text('p')).dy, 16.0);
      expect(tester.getTopLeft(find.text('s')).dy, 16.0);

      // layout is a row: [s text p]
      expect(tester.getTopLeft(find.text('s')).dx, 12.0);
      expect(
        tester.getTopRight(find.text('s')).dx,
        lessThanOrEqualTo(tester.getTopLeft(find.text('text')).dx),
      );
      expect(
        tester.getTopRight(find.text('text')).dx,
        lessThanOrEqualTo(tester.getTopLeft(find.text('p')).dx),
      );
    });

    testWidgets('InputDecorator contentPadding RTL layout', (WidgetTester tester) async {
      // LTR: content left edge is contentPadding.start: 40.0
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: const InputDecoration(
            contentPadding: EdgeInsetsDirectional.only(start: 40.0, top: 12.0, bottom: 12.0),
            labelText: 'label',
            hintText: 'hint',
            filled: true,
          ),
        ),
      );
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(tester.getTopLeft(find.text('text')).dx, 40.0);
      expect(tester.getTopLeft(find.text('label')).dx, 40.0);
      expect(tester.getTopLeft(find.text('hint')).dx, 40.0);

      // RTL: content right edge is 800 - contentPadding.start: 760.0.
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          isFocused: true, // label is floating, still adjusted for contentPadding
          textDirection: TextDirection.rtl,
          decoration: const InputDecoration(
            contentPadding: EdgeInsetsDirectional.only(start: 40.0, top: 12.0, bottom: 12.0),
            labelText: 'label',
            hintText: 'hint',
            filled: true,
          ),
        ),
      );
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(tester.getTopRight(find.text('text')).dx, 760.0);
      expect(tester.getTopRight(find.text('label')).dx, 760.0);
      expect(tester.getTopRight(find.text('hint')).dx, 760.0);
    });

    group('inputText width', () {
      testWidgets('outline textField', (WidgetTester tester) async {
        await tester.pumpWidget(
          buildInputDecoratorM2(decoration: const InputDecoration(border: OutlineInputBorder())),
        );
        expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
        expect(tester.getTopLeft(find.text('text')).dx, 12.0);
        expect(tester.getTopRight(find.text('text')).dx, 788.0);
      });
      testWidgets('outline textField with prefix and suffix icons', (WidgetTester tester) async {
        await tester.pumpWidget(
          buildInputDecoratorM2(
            decoration: const InputDecoration(
              border: OutlineInputBorder(),
              prefixIcon: Icon(Icons.visibility),
              suffixIcon: Icon(Icons.close),
            ),
          ),
        );
        expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
        expect(tester.getTopLeft(find.text('text')).dx, 48.0);
        expect(tester.getTopRight(find.text('text')).dx, 752.0);
      });
      testWidgets('filled textField', (WidgetTester tester) async {
        await tester.pumpWidget(
          buildInputDecoratorM2(decoration: const InputDecoration(filled: true)),
        );
        expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 48.0));
        expect(tester.getTopLeft(find.text('text')).dx, 12.0);
        expect(tester.getTopRight(find.text('text')).dx, 788.0);
      });
      testWidgets('filled textField with prefix and suffix icons', (WidgetTester tester) async {
        await tester.pumpWidget(
          buildInputDecoratorM2(
            decoration: const InputDecoration(
              filled: true,
              prefixIcon: Icon(Icons.visibility),
              suffixIcon: Icon(Icons.close),
            ),
          ),
        );
        expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 48.0));
        expect(tester.getTopLeft(find.text('text')).dx, 48.0);
        expect(tester.getTopRight(find.text('text')).dx, 752.0);
      });
    });

    group('floatingLabelAlignment', () {
      Widget buildInputDecoratorWithFloatingLabel({
        required TextDirection textDirection,
        required bool hasIcon,
        required FloatingLabelAlignment alignment,
        bool borderIsOutline = false,
      }) {
        return buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          textDirection: textDirection,
          decoration: InputDecoration(
            contentPadding: const EdgeInsetsDirectional.only(start: 40.0, top: 12.0, bottom: 12.0),
            floatingLabelAlignment: alignment,
            icon: hasIcon ? const Icon(Icons.insert_link) : null,
            labelText: 'label',
            hintText: 'hint',
            filled: true,
            border: borderIsOutline ? const OutlineInputBorder() : null,
          ),
        );
      }

      group('LTR with icon aligned', () {
        testWidgets('start', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecoratorWithFloatingLabel(
              textDirection: TextDirection.ltr,
              hasIcon: true,
              alignment: FloatingLabelAlignment.start,
              // borderIsOutline: false, (default)
            ),
          );
          // icon (40) + contentPadding (40)
          expect(tester.getTopLeft(find.text('label')).dx, 80.0);

          await tester.pumpWidget(
            buildInputDecoratorWithFloatingLabel(
              textDirection: TextDirection.ltr,
              hasIcon: true,
              alignment: FloatingLabelAlignment.start,
              borderIsOutline: true,
            ),
          );
          // icon (40) + contentPadding (40)
          expect(tester.getTopLeft(find.text('label')).dx, 80.0);
        });

        testWidgets('center', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecoratorWithFloatingLabel(
              textDirection: TextDirection.ltr,
              hasIcon: true,
              alignment: FloatingLabelAlignment.center,
              // borderIsOutline: false, (default)
            ),
          );
          // icon (40) + (decorator (800) - icon (40)) / 2
          expect(tester.getCenter(find.text('label')).dx, 420.0);

          await tester.pumpWidget(
            buildInputDecoratorWithFloatingLabel(
              textDirection: TextDirection.ltr,
              hasIcon: true,
              alignment: FloatingLabelAlignment.center,
              borderIsOutline: true,
            ),
          );
          // icon (40) + (decorator (800) - icon (40)) / 2
          expect(tester.getCenter(find.text('label')).dx, 420.0);
        });
      });

      group('LTR without icon aligned', () {
        testWidgets('start', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecoratorWithFloatingLabel(
              textDirection: TextDirection.ltr,
              hasIcon: false,
              alignment: FloatingLabelAlignment.start,
              // borderIsOutline: false, (default)
            ),
          );
          // contentPadding (40)
          expect(tester.getTopLeft(find.text('label')).dx, 40.0);

          await tester.pumpWidget(
            buildInputDecoratorWithFloatingLabel(
              textDirection: TextDirection.ltr,
              hasIcon: false,
              alignment: FloatingLabelAlignment.start,
              borderIsOutline: true,
            ),
          );
          // contentPadding (40)
          expect(tester.getTopLeft(find.text('label')).dx, 40.0);
        });

        testWidgets('center', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecoratorWithFloatingLabel(
              textDirection: TextDirection.ltr,
              hasIcon: false,
              alignment: FloatingLabelAlignment.center,
              // borderIsOutline: false, (default)
            ),
          );
          // decorator (800) / 2
          expect(tester.getCenter(find.text('label')).dx, 400.0);

          await tester.pumpWidget(
            buildInputDecoratorWithFloatingLabel(
              textDirection: TextDirection.ltr,
              hasIcon: false,
              alignment: FloatingLabelAlignment.center,
              borderIsOutline: true,
            ),
          );
          // decorator (800) / 2
          expect(tester.getCenter(find.text('label')).dx, 400.0);
        });
      });

      group('RTL with icon aligned', () {
        testWidgets('start', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecoratorWithFloatingLabel(
              textDirection: TextDirection.rtl,
              hasIcon: true,
              alignment: FloatingLabelAlignment.start,
              // borderIsOutline: false, (default)
            ),
          );
          // decorator (800) - icon (40) - contentPadding (40)
          expect(tester.getTopRight(find.text('label')).dx, 720.0);

          await tester.pumpWidget(
            buildInputDecoratorWithFloatingLabel(
              textDirection: TextDirection.rtl,
              hasIcon: true,
              alignment: FloatingLabelAlignment.start,
              borderIsOutline: true,
            ),
          );
          // decorator (800) - icon (40) - contentPadding (40)
          expect(tester.getTopRight(find.text('label')).dx, 720.0);
        });

        testWidgets('center', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecoratorWithFloatingLabel(
              textDirection: TextDirection.rtl,
              hasIcon: true,
              alignment: FloatingLabelAlignment.center,
              // borderIsOutline: false, (default)
            ),
          );
          // (decorator (800) / icon (40)) / 2
          expect(tester.getCenter(find.text('label')).dx, 380.0);

          await tester.pumpWidget(
            buildInputDecoratorWithFloatingLabel(
              textDirection: TextDirection.rtl,
              hasIcon: true,
              alignment: FloatingLabelAlignment.center,
              borderIsOutline: true,
            ),
          );
          // (decorator (800) / icon (40)) / 2
          expect(tester.getCenter(find.text('label')).dx, 380.0);
        });
      });

      group('RTL without icon aligned', () {
        testWidgets('start', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecoratorWithFloatingLabel(
              textDirection: TextDirection.rtl,
              hasIcon: false,
              alignment: FloatingLabelAlignment.start,
              // borderIsOutline: false, (default)
            ),
          );
          // decorator (800) - contentPadding (40)
          expect(tester.getTopRight(find.text('label')).dx, 760.0);

          await tester.pumpWidget(
            buildInputDecoratorWithFloatingLabel(
              textDirection: TextDirection.rtl,
              hasIcon: false,
              alignment: FloatingLabelAlignment.start,
              borderIsOutline: true,
            ),
          );
          // decorator (800) - contentPadding (40)
          expect(tester.getTopRight(find.text('label')).dx, 760.0);
        });

        testWidgets('center', (WidgetTester tester) async {
          await tester.pumpWidget(
            buildInputDecoratorWithFloatingLabel(
              textDirection: TextDirection.rtl,
              hasIcon: false,
              alignment: FloatingLabelAlignment.center,
              // borderIsOutline: false, (default)
            ),
          );
          // decorator (800) / 2
          expect(tester.getCenter(find.text('label')).dx, 400.0);

          await tester.pumpWidget(
            buildInputDecoratorWithFloatingLabel(
              textDirection: TextDirection.rtl,
              hasIcon: false,
              alignment: FloatingLabelAlignment.center,
              borderIsOutline: true,
            ),
          );
          // decorator (800) / 2
          expect(tester.getCenter(find.text('label')).dx, 400.0);
        });
      });
    });

    testWidgets('InputDecorator prefix/suffix dense layout', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          isFocused: true,
          decoration: const InputDecoration(
            isDense: true,
            prefixText: 'p',
            suffixText: 's',
            filled: true,
          ),
        ),
      );

      // Overall height for this InputDecorator is 32dps:
      //    8 - top padding
      //   16 - input text (font size 16dps)
      //    8 - bottom padding
      //
      // The only difference from normal layout for this case is that the
      // padding above and below the prefix, input text, suffix, is 8 instead of 12.

      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 32.0));
      expect(tester.getSize(find.text('text')).height, 16.0);
      expect(tester.getSize(find.text('p')).height, 16.0);
      expect(tester.getSize(find.text('s')).height, 16.0);
      expect(tester.getTopLeft(find.text('text')).dy, 8.0);
      expect(tester.getTopLeft(find.text('p')).dy, 8.0);
      expect(tester.getTopLeft(find.text('p')).dx, 12.0);
      expect(tester.getTopLeft(find.text('s')).dy, 8.0);
      expect(tester.getTopRight(find.text('s')).dx, 788.0);

      // layout is a row: [p text s]
      expect(tester.getTopLeft(find.text('p')).dx, 12.0);
      expect(
        tester.getTopRight(find.text('p')).dx,
        lessThanOrEqualTo(tester.getTopLeft(find.text('text')).dx),
      );
      expect(
        tester.getTopRight(find.text('text')).dx,
        lessThanOrEqualTo(tester.getTopLeft(find.text('s')).dx),
      );

      expect(getBorderBottom(tester), 32.0);
      expect(getBorderWeight(tester), 2.0);
    });

    testWidgets('InputDecorator with empty InputDecoration', (WidgetTester tester) async {
      await tester.pumpWidget(buildInputDecoratorM2());

      // Overall height for this InputDecorator is 40dps:
      //   12 - top padding
      //   16 - input text (font size 16dps)
      //   12 - bottom padding

      expect(
        tester.getSize(find.byType(InputDecorator)),
        const Size(800.0, kMinInteractiveDimension),
      ); // 40 bumped up to minimum.
      expect(tester.getSize(find.text('text')).height, 16.0);
      expect(tester.getTopLeft(find.text('text')).dy, 16.0);
      expect(getBorderBottom(tester), kMinInteractiveDimension); // 40 bumped up to minimum.
      expect(getBorderWeight(tester), 1.0);
    });

    testWidgets('contentPadding smaller than kMinInteractiveDimension', (
      WidgetTester tester,
    ) async {
      // Regression test for https://github.com/flutter/flutter/issues/42449
      const double verticalPadding = 1.0;
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default),
          // isFocused: false (default)
          decoration: const InputDecoration(
            hintText: 'hint',
            contentPadding: EdgeInsets.symmetric(vertical: verticalPadding),
            isDense: true,
          ),
        ),
      );

      // The overall height is 18dps. This is shorter than
      // kMinInteractiveDimension, but because isDense is true, the minimum is
      // ignored.
      //   16 - input text (font size 16dps)
      //    2 - total vertical padding

      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 18.0));
      expect(tester.getSize(find.text('text')).height, 16.0);
      expect(tester.getTopLeft(find.text('text')).dy, 1.0);
      expect(getOpacity(tester, 'hint'), 0.0);
      expect(getBorderWeight(tester), 1.0);

      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default),
          // isFocused: false (default)
          decoration: const InputDecoration.collapsed(
            hintText: 'hint',
            // InputDecoration.collapsed does not support contentPadding
          ),
        ),
      );

      // The overall height is 16dps. This is shorter than
      // kMinInteractiveDimension, but because isCollapsed is true, the minimum is
      // ignored. There is no padding at all, because isCollapsed doesn't support
      // contentPadding.
      //   16 - input text (font size 16dps)

      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 16.0));
      expect(tester.getSize(find.text('text')).height, 16.0);
      expect(tester.getTopLeft(find.text('text')).dy, 0.0);
      expect(getOpacity(tester, 'hint'), 0.0);
      expect(getBorderWeight(tester), 1.0);

      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default),
          // isFocused: false (default)
          decoration: const InputDecoration(
            hintText: 'hint',
            contentPadding: EdgeInsets.symmetric(vertical: verticalPadding),
          ),
        ),
      );

      // The requested overall height is 18dps, however the minimum height is
      // kMinInteractiveDimension because neither isDense or isCollapsed are true.
      //   16 - input text (font size 16dps)
      //    2 - total vertical padding

      expect(
        tester.getSize(find.byType(InputDecorator)),
        const Size(800.0, kMinInteractiveDimension),
      );
      expect(tester.getSize(find.text('text')).height, 16.0);
      expect(tester.getTopLeft(find.text('text')).dy, 16.0);
      expect(getOpacity(tester, 'hint'), 0.0);
      expect(getBorderWeight(tester), 0.0);
    });

    testWidgets('InputDecorator.collapsed', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default),
          // isFocused: false (default)
          decoration: const InputDecoration.collapsed(hintText: 'hint'),
        ),
      );

      // Overall height for this InputDecorator is 16dps. There is no minimum
      // height when InputDecoration.collapsed is used.
      //   16 - input text (font size 16dps)

      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 16.0));
      expect(tester.getSize(find.text('text')).height, 16.0);
      expect(tester.getTopLeft(find.text('text')).dy, 0.0);
      expect(getOpacity(tester, 'hint'), 0.0);
      expect(getBorderWeight(tester), 0.0);

      // The hint should appear
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          isFocused: true,
          decoration: const InputDecoration.collapsed(hintText: 'hint'),
        ),
      );
      await tester.pumpAndSettle();

      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 16.0));
      expect(tester.getSize(find.text('text')).height, 16.0);
      expect(tester.getTopLeft(find.text('text')).dy, 0.0);
      expect(tester.getSize(find.text('hint')).height, 16.0);
      expect(tester.getTopLeft(find.text('hint')).dy, 0.0);
      expect(getBorderWeight(tester), 0.0);
    });

    testWidgets('InputDecorator with baseStyle', (WidgetTester tester) async {
      // Setting the baseStyle of the InputDecoration and the style of the input
      // text child to a smaller font reduces the InputDecoration's vertical size.
      const TextStyle style = TextStyle(fontSize: 10.0);
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          baseStyle: style,
          decoration: const InputDecoration(hintText: 'hint', labelText: 'label'),
          child: const Text('text', style: style),
        ),
      );

      // Overall height for this InputDecorator is 45.5dps. When the label is
      // floating the layout is:
      //
      //    12  - top padding
      //    7.5 - floating label (font size 10dps * 0.75 = 7.5)
      //    4   - floating label / input text gap
      //   10   - input text (font size 10dps)
      //   12   - bottom padding
      //
      // When the label is not floating, it's vertically centered.
      //
      //   17.75 - top padding
      //      10 - label (font size 10dps)
      //   17.75 - bottom padding (empty input text still appears here)

      expect(
        tester.getSize(find.byType(InputDecorator)),
        const Size(800.0, kMinInteractiveDimension),
      ); // 45.5 bumped up to minimum.
      expect(tester.getSize(find.text('hint')).height, 10.0);
      expect(tester.getSize(find.text('label')).height, 10.0);
      expect(tester.getSize(find.text('text')).height, 10.0);
      expect(tester.getTopLeft(find.text('hint')).dy, 24.75);
      expect(tester.getTopLeft(find.text('label')).dy, 19.0);
      expect(tester.getTopLeft(find.text('text')).dy, 24.75);
    });

    testWidgets('InputDecorator with empty style overrides', (WidgetTester tester) async {
      // Same as not specifying any style overrides
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: const InputDecoration(
            labelText: 'label',
            hintText: 'hint',
            helperText: 'helper',
            counterText: 'counter',
            labelStyle: TextStyle(),
            hintStyle: TextStyle(),
            errorStyle: TextStyle(),
            helperStyle: TextStyle(),
            filled: true,
          ),
        ),
      );

      // Overall height for this InputDecorator is 76dps. When the label is
      // floating the layout is:
      //   12 - top padding
      //   12 - floating label (font size 16dps * 0.75 = 12)
      //    4 - floating label / input text gap
      //   16 - input text (font size 16dps)
      //   12 - bottom padding
      //    8 - below the border padding
      //   12 - help/error/counter text (font size 12dps)

      // Label is floating because isEmpty is false.
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 76.0));
      expect(tester.getTopLeft(find.text('text')).dy, 28.0);
      expect(tester.getBottomLeft(find.text('text')).dy, 44.0);
      expect(tester.getTopLeft(find.text('label')).dy, 12.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 24.0);
      expect(getBorderBottom(tester), 56.0);
      expect(getBorderWeight(tester), 1.0);
      expect(tester.getTopLeft(find.text('helper')), const Offset(12.0, 64.0));
      expect(tester.getTopRight(find.text('counter')), const Offset(788.0, 64.0));
    });

    testWidgets('InputDecoration outline shape with no border and no floating placeholder', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isFocused: false (default)
          isEmpty: true,
          decoration: const InputDecoration(
            border: OutlineInputBorder(borderSide: BorderSide.none),
            floatingLabelBehavior: FloatingLabelBehavior.never,
            labelText: 'label',
          ),
        ),
      );

      // Overall height for this InputDecorator is 56dps. Layout is:
      //   20 - top padding
      //   16 - label (font size 16dps)
      //   20 - bottom padding
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(tester.getTopLeft(find.text('label')).dy, 20.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 36.0);
      expect(getBorderBottom(tester), 56.0);
      expect(getBorderWeight(tester), 0.0);
    });

    testWidgets(
      'InputDecoration outline shape with no border and no floating placeholder not empty',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          buildInputDecoratorM2(
            // isEmpty: false (default)
            // isFocused: false (default)
            decoration: const InputDecoration(
              border: OutlineInputBorder(borderSide: BorderSide.none),
              floatingLabelBehavior: FloatingLabelBehavior.never,
              labelText: 'label',
            ),
          ),
        );

        // Overall height for this InputDecorator is 56dps. Layout is:
        //   20 - top padding
        //   16 - label (font size 16dps)
        //   20 - bottom padding
        //    expect(tester.widget<Text>(find.text('prefix')).style.color, prefixStyle.color);
        expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
        expect(tester.getTopLeft(find.text('label')).dy, 20.0);
        expect(tester.getBottomLeft(find.text('label')).dy, 36.0);
        expect(getBorderBottom(tester), 56.0);
        expect(getBorderWeight(tester), 0.0);

        // The label should not be seen.
        expect(getOpacity(tester, 'label'), 0.0);
      },
    );

    testWidgets('InputDecorationTheme outline border', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true, // label appears, vertically centered
          // isFocused: false (default)
          inputDecorationTheme: const InputDecorationTheme(border: OutlineInputBorder()),
          decoration: const InputDecoration(labelText: 'label'),
        ),
      );

      // Overall height for this InputDecorator is 56dps. Layout is:
      //   20 - top padding
      //   16 - label (font size 16dps)
      //   20 - bottom padding
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(tester.getTopLeft(find.text('label')).dy, 20.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 36.0);
      expect(getBorderBottom(tester), 56.0);
      expect(getBorderWeight(tester), 1.0);
    });

    testWidgets('InputDecorationTheme outline border, dense layout', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true, // label appears, vertically centered
          // isFocused: false (default)
          inputDecorationTheme: const InputDecorationTheme(
            border: OutlineInputBorder(),
            isDense: true,
          ),
          decoration: const InputDecoration(labelText: 'label', hintText: 'hint'),
        ),
      );

      // Overall height for this InputDecorator is 56dps. Layout is:
      //   16 - top padding
      //   16 - label (font size 16dps)
      //   16 - bottom padding
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 48.0));
      expect(tester.getTopLeft(find.text('label')).dy, 16.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 32.0);
      expect(getBorderBottom(tester), 48.0);
      expect(getBorderWeight(tester), 1.0);
    });

    testWidgets('InputDecorationTheme style overrides', (WidgetTester tester) async {
      const TextStyle defaultStyle = TextStyle(fontSize: 16.0);
      final TextStyle labelStyle = defaultStyle.merge(const TextStyle(color: Colors.red));
      final TextStyle hintStyle = defaultStyle.merge(const TextStyle(color: Colors.green));
      final TextStyle prefixStyle = defaultStyle.merge(const TextStyle(color: Colors.blue));
      final TextStyle suffixStyle = defaultStyle.merge(const TextStyle(color: Colors.purple));

      const TextStyle style12 = TextStyle(fontSize: 12.0);
      final TextStyle helperStyle = style12.merge(const TextStyle(color: Colors.orange));
      final TextStyle counterStyle = style12.merge(const TextStyle(color: Colors.orange));

      // This test also verifies that the default InputDecorator provides a
      // "small concession to backwards compatibility" by not padding on
      // the left and right. If filled is true or an outline border is
      // provided then the horizontal padding is included.

      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true, // label appears, vertically centered
          // isFocused: false (default)
          inputDecorationTheme: InputDecorationTheme(
            labelStyle: labelStyle,
            hintStyle: hintStyle,
            prefixStyle: prefixStyle,
            suffixStyle: suffixStyle,
            helperStyle: helperStyle,
            counterStyle: counterStyle,
            // filled: false (default) - don't pad by left/right 12dps
          ),
          decoration: const InputDecoration(
            labelText: 'label',
            hintText: 'hint',
            prefixText: 'prefix',
            suffixText: 'suffix',
            helperText: 'helper',
            counterText: 'counter',
          ),
        ),
      );

      // Overall height for this InputDecorator is 76dps. Layout is:
      //   12 - top padding
      //   12 - floating label (font size 16dps * 0.75 = 12)
      //    4 - floating label / input text gap
      //   16 - prefix/hint/input/suffix text (font size 16dps)
      //   12 - bottom padding
      //    8 - below the border padding
      //   12 - help/error/counter text (font size 12dps)
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 76.0));
      expect(tester.getTopLeft(find.text('label')).dy, 20.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 36.0);
      expect(getBorderBottom(tester), 56.0);
      expect(getBorderWeight(tester), 1.0);
      expect(tester.getTopLeft(find.text('helper')), const Offset(0.0, 64.0));
      expect(tester.getTopRight(find.text('counter')), const Offset(800.0, 64.0));

      // Verify that the styles were passed along
      expect(tester.widget<Text>(find.text('hint')).style!.color, hintStyle.color);
      expect(tester.widget<Text>(find.text('prefix')).style!.color, prefixStyle.color);
      expect(tester.widget<Text>(find.text('suffix')).style!.color, suffixStyle.color);
      expect(tester.widget<Text>(find.text('helper')).style!.color, helperStyle.color);
      expect(tester.widget<Text>(find.text('counter')).style!.color, counterStyle.color);
      expect(getLabelStyle(tester).color, labelStyle.color);
    });

    testWidgets('InputDecorationTheme style overrides (focused)', (WidgetTester tester) async {
      const TextStyle defaultStyle = TextStyle(fontSize: 16.0);
      final TextStyle labelStyle = defaultStyle.merge(const TextStyle(color: Colors.red));
      final TextStyle floatingLabelStyle = defaultStyle.merge(
        const TextStyle(color: Colors.indigo),
      );
      final TextStyle hintStyle = defaultStyle.merge(const TextStyle(color: Colors.green));
      final TextStyle prefixStyle = defaultStyle.merge(const TextStyle(color: Colors.blue));
      final TextStyle suffixStyle = defaultStyle.merge(const TextStyle(color: Colors.purple));

      const TextStyle style12 = TextStyle(fontSize: 12.0);
      final TextStyle helperStyle = style12.merge(const TextStyle(color: Colors.orange));
      final TextStyle counterStyle = style12.merge(const TextStyle(color: Colors.orange));

      // This test also verifies that the default InputDecorator provides a
      // "small concession to backwards compatibility" by not padding on
      // the left and right. If filled is true or an outline border is
      // provided then the horizontal padding is included.

      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          isFocused: true, // Label appears floating above input field.
          inputDecorationTheme: InputDecorationTheme(
            labelStyle: labelStyle,
            floatingLabelStyle: floatingLabelStyle,
            hintStyle: hintStyle,
            prefixStyle: prefixStyle,
            suffixStyle: suffixStyle,
            helperStyle: helperStyle,
            counterStyle: counterStyle,
            // filled: false (default) - don't pad by left/right 12dps
          ),
          decoration: const InputDecoration(
            labelText: 'label',
            hintText: 'hint',
            prefixText: 'prefix',
            suffixText: 'suffix',
            helperText: 'helper',
            counterText: 'counter',
          ),
        ),
      );

      // Overall height for this InputDecorator is 76dps. Layout is:
      //   12 - top padding
      //   12 - floating label (font size 16dps * 0.75 = 12)
      //    4 - floating label / input text gap
      //   16 - prefix/hint/input/suffix text (font size 16dps)
      //   12 - bottom padding
      //    8 - below the border padding
      //   12 - help/error/counter text (font size 12dps)
      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 76.0));
      expect(tester.getTopLeft(find.text('label')).dy, 12.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 24.0);
      expect(getBorderBottom(tester), 56.0);
      expect(getBorderWeight(tester), 2.0);
      expect(tester.getTopLeft(find.text('helper')), const Offset(0.0, 64.0));
      expect(tester.getTopRight(find.text('counter')), const Offset(800.0, 64.0));

      // Verify that the styles were passed along
      expect(tester.widget<Text>(find.text('hint')).style!.color, hintStyle.color);
      expect(tester.widget<Text>(find.text('prefix')).style!.color, prefixStyle.color);
      expect(tester.widget<Text>(find.text('suffix')).style!.color, suffixStyle.color);
      expect(tester.widget<Text>(find.text('helper')).style!.color, helperStyle.color);
      expect(tester.widget<Text>(find.text('counter')).style!.color, counterStyle.color);
      expect(getLabelStyle(tester).color, floatingLabelStyle.color);
    });

    testWidgets('InputDecorator.debugDescribeChildren', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          decoration: const InputDecoration(
            icon: Text('icon'),
            labelText: 'label',
            hintText: 'hint',
            prefixText: 'prefix',
            suffixText: 'suffix',
            prefixIcon: Text('prefixIcon'),
            suffixIcon: Text('suffixIcon'),
            helperText: 'helper',
            counterText: 'counter',
          ),
          child: const Text('text'),
        ),
      );

      final RenderObject renderer = tester.renderObject(find.byType(InputDecorator));
      final Iterable<String> nodeNames = renderer.debugDescribeChildren().map(
        (DiagnosticsNode node) => node.name!,
      );
      expect(
        nodeNames,
        unorderedEquals(<String>[
          'container',
          'counter',
          'helperError',
          'hint',
          'icon',
          'input',
          'label',
          'prefix',
          'prefixIcon',
          'suffix',
          'suffixIcon',
        ]),
      );

      final Set<Object> nodeValues = Set<Object>.from(
        renderer.debugDescribeChildren().map<Object>((DiagnosticsNode node) => node.value!),
      );
      expect(nodeValues.length, 11);
    });

    testWidgets('InputDecorator with empty border and label', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/14165
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: const InputDecoration(labelText: 'label', border: InputBorder.none),
        ),
      );

      expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
      expect(getBorderWeight(tester), 0.0);
      expect(tester.getTopLeft(find.text('label')).dy, 12.0);
      expect(tester.getBottomLeft(find.text('label')).dy, 24.0);
    });

    testWidgets('InputDecorator OutlineInputBorder fillColor is clipped by border', (
      WidgetTester tester,
    ) async {
      // This is a regression test for https://github.com/flutter/flutter/issues/15742

      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: const InputDecoration(
            filled: true,
            fillColor: Color(0xFF00FF00),
            border: OutlineInputBorder(borderRadius: BorderRadius.all(Radius.circular(12.0))),
          ),
        ),
      );

      final RenderBox box = tester.renderObject(find.byType(InputDecorator));

      // Fill is the border's outer path, a rounded rectangle
      expect(
        box,
        paints..path(
          style: PaintingStyle.fill,
          color: const Color(0xFF00FF00),
          includes: <Offset>[const Offset(800.0 / 2.0, 56 / 2.0)],
          excludes: <Offset>[
            const Offset(1.0, 6.0), // outside the rounded corner, top left
            const Offset(800.0 - 1.0, 6.0), // top right
            const Offset(1.0, 56.0 - 6.0), // bottom left
            const Offset(800 - 1.0, 56.0 - 6.0), // bottom right
          ],
        ),
      );

      // Border outline. The rrect is the -center- of the 1.0 stroked outline.
      expect(
        box,
        paints..rrect(
          style: PaintingStyle.stroke,
          strokeWidth: 1.0,
          rrect: RRect.fromLTRBR(0.5, 0.5, 799.5, 55.5, const Radius.circular(11.5)),
        ),
      );
    });

    testWidgets('InputDecorator UnderlineInputBorder fillColor is clipped by border', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: const InputDecoration(
            filled: true,
            fillColor: Color(0xFF00FF00),
            border: UnderlineInputBorder(
              borderRadius: BorderRadius.only(
                bottomLeft: Radius.circular(12.0),
                bottomRight: Radius.circular(12.0),
              ),
            ),
          ),
        ),
      );

      final RenderBox box = tester.renderObject(find.byType(InputDecorator));

      // Fill is the border's outer path, a rounded rectangle
      expect(
        box,
        paints..drrect(
          style: PaintingStyle.fill,
          inner: RRect.fromLTRBAndCorners(
            0.0,
            0.0,
            800.0,
            47.0,
            bottomRight: const Radius.elliptical(12.0, 11.0),
            bottomLeft: const Radius.elliptical(12.0, 11.0),
          ),
          outer: RRect.fromLTRBAndCorners(
            0.0,
            0.0,
            800.0,
            48.0,
            bottomRight: const Radius.elliptical(12.0, 12.0),
            bottomLeft: const Radius.elliptical(12.0, 12.0),
          ),
        ),
      );
    });

    testWidgets('InputDecorator OutlineBorder focused label with icon', (
      WidgetTester tester,
    ) async {
      // This is a regression test for https://github.com/flutter/flutter/issues/82321
      Widget buildFrame(TextDirection textDirection) {
        return MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Scaffold(
            body: Container(
              padding: const EdgeInsets.all(16.0),
              alignment: Alignment.center,
              child: Directionality(
                textDirection: textDirection,
                child: RepaintBoundary(
                  child: InputDecorator(
                    isFocused: true,
                    isEmpty: true,
                    decoration: InputDecoration(
                      filled: true,
                      fillColor: const Color(0xFF00FF00),
                      labelText: 'label text',
                      border: OutlineInputBorder(
                        borderRadius: BorderRadius.circular(30.0),
                        gapPadding: 0.0,
                      ),
                    ),
                  ),
                ),
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildFrame(TextDirection.ltr));
      await expectLater(
        find.byType(InputDecorator),
        matchesGoldenFile('m2_input_decorator.outline_label.ltr.png'),
      );

      await tester.pumpWidget(buildFrame(TextDirection.rtl));
      await expectLater(
        find.byType(InputDecorator),
        matchesGoldenFile('m2_input_decorator.outline_label.rtl.png'),
      );
    });

    testWidgets('InputDecorator OutlineBorder focused label with icon', (
      WidgetTester tester,
    ) async {
      // Regression test for https://github.com/flutter/flutter/issues/18111

      Widget buildFrame(TextDirection textDirection) {
        return MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Scaffold(
            body: Container(
              padding: const EdgeInsets.all(16.0),
              alignment: Alignment.center,
              child: Directionality(
                textDirection: textDirection,
                child: const RepaintBoundary(
                  child: InputDecorator(
                    isFocused: true,
                    isEmpty: true,
                    decoration: InputDecoration(
                      icon: Icon(Icons.insert_link),
                      labelText: 'primaryLink',
                      hintText: 'Primary link to story',
                      border: OutlineInputBorder(),
                    ),
                  ),
                ),
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildFrame(TextDirection.ltr));
      await expectLater(
        find.byType(InputDecorator),
        matchesGoldenFile('m2_input_decorator.outline_icon_label.ltr.png'),
      );

      await tester.pumpWidget(buildFrame(TextDirection.rtl));
      await expectLater(
        find.byType(InputDecorator),
        matchesGoldenFile('m2_input_decorator.outline_icon_label.rtl.png'),
      );
    });

    testWidgets('InputDecorator draws and animates hoverColor', (WidgetTester tester) async {
      const Color fillColor = Color(0x0A000000);
      const Color hoverColor = Color(0xFF00FF00);
      const Color disabledColor = Color(0x05000000);
      const Color enabledBorderColor = Color(0x61000000);

      Future<void> pumpDecorator({
        required bool hovering,
        bool enabled = true,
        bool filled = true,
      }) async {
        return tester.pumpWidget(
          buildInputDecoratorM2(
            isHovering: hovering,
            decoration: InputDecoration(
              enabled: enabled,
              filled: filled,
              hoverColor: hoverColor,
              disabledBorder: const OutlineInputBorder(
                borderSide: BorderSide(color: disabledColor),
              ),
              border: const OutlineInputBorder(borderSide: BorderSide(color: enabledBorderColor)),
            ),
          ),
        );
      }

      // Test filled text field.
      await pumpDecorator(hovering: false);
      expect(getContainerColor(tester), isSameColorAs(fillColor));
      await tester.pump(const Duration(seconds: 10));
      expect(getContainerColor(tester), isSameColorAs(fillColor));

      await pumpDecorator(hovering: true);
      expect(getContainerColor(tester), isSameColorAs(fillColor));
      await tester.pump(const Duration(milliseconds: 15));
      expect(getContainerColor(tester), isSameColorAs(hoverColor));

      await pumpDecorator(hovering: false);
      expect(getContainerColor(tester), isSameColorAs(hoverColor));
      await tester.pump(const Duration(milliseconds: 15));
      expect(getContainerColor(tester), isSameColorAs(fillColor));

      await pumpDecorator(hovering: false, enabled: false);
      expect(getContainerColor(tester), isSameColorAs(disabledColor));
      await tester.pump(const Duration(seconds: 10));
      expect(getContainerColor(tester), isSameColorAs(disabledColor));

      await pumpDecorator(hovering: true, enabled: false);
      expect(getContainerColor(tester), isSameColorAs(disabledColor));
      await tester.pump(const Duration(seconds: 10));
      expect(getContainerColor(tester), isSameColorAs(disabledColor));

      // Test outline text field.
      const Color blendedHoverColor = Color(0x74004400);
      await pumpDecorator(hovering: false, filled: false);
      await tester.pumpAndSettle();
      expect(getBorderColor(tester), isSameColorAs(enabledBorderColor));
      await tester.pump(const Duration(seconds: 10));
      expect(getBorderColor(tester), isSameColorAs(enabledBorderColor));

      await pumpDecorator(hovering: true, filled: false);
      expect(getBorderColor(tester), isSameColorAs(enabledBorderColor));
      await tester.pump(const Duration(milliseconds: 167));
      expect(getBorderColor(tester), isSameColorAs(blendedHoverColor));

      await pumpDecorator(hovering: false, filled: false);
      expect(getBorderColor(tester), isSameColorAs(blendedHoverColor));
      await tester.pump(const Duration(milliseconds: 167));
      expect(getBorderColor(tester), isSameColorAs(enabledBorderColor));

      await pumpDecorator(hovering: false, filled: false, enabled: false);
      expect(getBorderColor(tester), isSameColorAs(enabledBorderColor));
      await tester.pump(const Duration(milliseconds: 167));
      expect(getBorderColor(tester), isSameColorAs(disabledColor));

      await pumpDecorator(hovering: true, filled: false, enabled: false);
      expect(getBorderColor(tester), isSameColorAs(disabledColor));
      await tester.pump(const Duration(seconds: 10));
      expect(getBorderColor(tester), isSameColorAs(disabledColor));
    });

    testWidgets('InputDecorator draws and animates focusColor', (WidgetTester tester) async {
      const Color focusColor = Color(0xFF0000FF);
      const Color disabledColor = Color(0x05000000);
      const Color enabledBorderColor = Color(0x61000000);

      Future<void> pumpDecorator({
        required bool focused,
        bool enabled = true,
        bool filled = true,
      }) async {
        return tester.pumpWidget(
          buildInputDecoratorM2(
            isFocused: focused,
            decoration: InputDecoration(
              enabled: enabled,
              filled: filled,
              focusColor: focusColor,
              focusedBorder: const OutlineInputBorder(borderSide: BorderSide(color: focusColor)),
              disabledBorder: const OutlineInputBorder(
                borderSide: BorderSide(color: disabledColor),
              ),
              border: const OutlineInputBorder(borderSide: BorderSide(color: enabledBorderColor)),
            ),
          ),
        );
      }

      // Test outline text field default border.
      await pumpDecorator(focused: false, filled: false);
      await tester.pumpAndSettle();
      expect(getBorderColor(tester), equals(enabledBorderColor));
      await tester.pump(const Duration(seconds: 10));
      expect(getBorderColor(tester), equals(enabledBorderColor));

      await pumpDecorator(focused: true, filled: false);
      expect(getBorderColor(tester), equals(enabledBorderColor));
      await tester.pump(const Duration(milliseconds: 167));
      expect(getBorderColor(tester), equals(focusColor));

      await pumpDecorator(focused: false, filled: false);
      expect(getBorderColor(tester), equals(focusColor));
      await tester.pump(const Duration(milliseconds: 167));
      expect(getBorderColor(tester), equals(enabledBorderColor));

      await pumpDecorator(focused: false, filled: false, enabled: false);
      expect(getBorderColor(tester), equals(enabledBorderColor));
      await tester.pump(const Duration(milliseconds: 167));
      expect(getBorderColor(tester), equals(disabledColor));

      await pumpDecorator(focused: true, filled: false, enabled: false);
      expect(getBorderColor(tester), equals(disabledColor));
      await tester.pump(const Duration(seconds: 10));
      expect(getBorderColor(tester), equals(disabledColor));
    });

    testWidgets('InputDecorator withdraws label when not empty or focused', (
      WidgetTester tester,
    ) async {
      Future<void> pumpDecorator({
        required bool focused,
        bool enabled = true,
        bool filled = false,
        bool empty = true,
        bool directional = false,
      }) async {
        return tester.pumpWidget(
          buildInputDecoratorM2(
            isEmpty: empty,
            isFocused: focused,
            decoration: InputDecoration(
              labelText: 'Label',
              enabled: enabled,
              filled: filled,
              focusedBorder: const OutlineInputBorder(),
              disabledBorder: const OutlineInputBorder(),
              border: const OutlineInputBorder(),
            ),
          ),
        );
      }

      await pumpDecorator(focused: false);
      await tester.pumpAndSettle();
      const Size labelSize = Size(80, 16);
      expect(getLabelRect(tester).topLeft, equals(const Offset(12, 20)));
      expect(getLabelRect(tester).size, equals(labelSize));

      await pumpDecorator(focused: false, empty: false);
      await tester.pumpAndSettle();
      expect(getLabelRect(tester).topLeft, equals(const Offset(12, -5.5)));
      expect(getLabelRect(tester).size, equals(labelSize * 0.75));

      await pumpDecorator(focused: true);
      await tester.pumpAndSettle();
      expect(getLabelRect(tester).topLeft, equals(const Offset(12, -5.5)));
      expect(getLabelRect(tester).size, equals(labelSize * 0.75));

      await pumpDecorator(focused: true, empty: false);
      await tester.pumpAndSettle();
      expect(getLabelRect(tester).topLeft, equals(const Offset(12, -5.5)));
      expect(getLabelRect(tester).size, equals(labelSize * 0.75));

      await pumpDecorator(focused: false, enabled: false);
      await tester.pumpAndSettle();
      expect(getLabelRect(tester).topLeft, equals(const Offset(12, 20)));
      expect(getLabelRect(tester).size, equals(labelSize));

      await pumpDecorator(focused: false, empty: false, enabled: false);
      await tester.pumpAndSettle();
      expect(getLabelRect(tester).topLeft, equals(const Offset(12, -5.5)));
      expect(getLabelRect(tester).size, equals(labelSize * 0.75));

      // Focused and disabled happens with NavigationMode.directional.
      await pumpDecorator(focused: true, enabled: false);
      await tester.pumpAndSettle();
      expect(getLabelRect(tester).topLeft, equals(const Offset(12, 20)));
      expect(getLabelRect(tester).size, equals(labelSize));

      await pumpDecorator(focused: true, empty: false, enabled: false);
      await tester.pumpAndSettle();
      expect(getLabelRect(tester).topLeft, equals(const Offset(12, -5.5)));
      expect(getLabelRect(tester).size, equals(labelSize * 0.75));
    });

    testWidgets('InputDecoration default border uses colorScheme', (WidgetTester tester) async {
      final ThemeData theme = ThemeData.light(useMaterial3: false);
      final Color enabledColor = theme.colorScheme.onSurface.withOpacity(0.38);
      final Color disabledColor = theme.disabledColor;
      final Color hoverColor = Color.alphaBlend(theme.hoverColor.withOpacity(0.12), enabledColor);

      // Enabled
      await tester.pumpWidget(buildInputDecoratorM2(theme: theme));
      await tester.pumpAndSettle();
      expect(getBorderColor(tester), enabledColor);

      // Filled
      await tester.pumpWidget(
        buildInputDecoratorM2(theme: theme, decoration: const InputDecoration(filled: true)),
      );
      await tester.pumpAndSettle();
      expect(getBorderColor(tester), theme.hintColor);

      // Hovering
      await tester.pumpWidget(buildInputDecoratorM2(theme: theme, isHovering: true));
      await tester.pumpAndSettle();
      expect(getBorderColor(tester), hoverColor);

      // Focused
      await tester.pumpWidget(buildInputDecoratorM2(theme: theme, isFocused: true));
      await tester.pumpAndSettle();
      expect(getBorderColor(tester), theme.colorScheme.primary);

      // Error
      await tester.pumpWidget(
        buildInputDecoratorM2(theme: theme, decoration: const InputDecoration(errorText: 'Nope')),
      );
      await tester.pumpAndSettle();
      expect(getBorderColor(tester), theme.colorScheme.error);

      // Disabled
      await tester.pumpWidget(
        buildInputDecoratorM2(theme: theme, decoration: const InputDecoration(enabled: false)),
      );
      await tester.pumpAndSettle();
      expect(getBorderColor(tester), disabledColor);

      // Disabled, filled
      await tester.pumpWidget(
        buildInputDecoratorM2(
          theme: theme,
          decoration: const InputDecoration(enabled: false, filled: true),
        ),
      );
      await tester.pumpAndSettle();
      expect(getBorderColor(tester), Colors.transparent);
    });

    testWidgets('InputDecoration borders', (WidgetTester tester) async {
      const InputBorder errorBorder = OutlineInputBorder(
        borderSide: BorderSide(color: Colors.red, width: 1.5),
      );
      const InputBorder focusedBorder = OutlineInputBorder(
        borderSide: BorderSide(color: Colors.green, width: 4.0),
      );
      const InputBorder focusedErrorBorder = OutlineInputBorder(
        borderSide: BorderSide(color: Colors.teal, width: 5.0),
      );
      const InputBorder disabledBorder = OutlineInputBorder(
        borderSide: BorderSide(color: Colors.grey, width: 0.0),
      );
      const InputBorder enabledBorder = OutlineInputBorder(
        borderSide: BorderSide(color: Colors.blue, width: 2.5),
      );

      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isFocused: false (default)
          decoration: const InputDecoration(
            // errorText: null (default)
            // enabled: true (default)
            errorBorder: errorBorder,
            focusedBorder: focusedBorder,
            focusedErrorBorder: focusedErrorBorder,
            disabledBorder: disabledBorder,
            enabledBorder: enabledBorder,
          ),
        ),
      );
      expect(getBorder(tester), enabledBorder);

      await tester.pumpWidget(
        buildInputDecoratorM2(
          isFocused: true,
          decoration: const InputDecoration(
            // errorText: null (default)
            // enabled: true (default)
            errorBorder: errorBorder,
            focusedBorder: focusedBorder,
            focusedErrorBorder: focusedErrorBorder,
            disabledBorder: disabledBorder,
            enabledBorder: enabledBorder,
          ),
        ),
      );
      await tester.pumpAndSettle(); // border changes are animated
      expect(getBorder(tester), focusedBorder);

      await tester.pumpWidget(
        buildInputDecoratorM2(
          isFocused: true,
          decoration: const InputDecoration(
            errorText: 'error',
            // enabled: true (default)
            errorBorder: errorBorder,
            focusedBorder: focusedBorder,
            focusedErrorBorder: focusedErrorBorder,
            disabledBorder: disabledBorder,
            enabledBorder: enabledBorder,
          ),
        ),
      );
      await tester.pumpAndSettle(); // border changes are animated
      expect(getBorder(tester), focusedErrorBorder);

      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isFocused: false (default)
          decoration: const InputDecoration(
            errorText: 'error',
            // enabled: true (default)
            errorBorder: errorBorder,
            focusedBorder: focusedBorder,
            focusedErrorBorder: focusedErrorBorder,
            disabledBorder: disabledBorder,
            enabledBorder: enabledBorder,
          ),
        ),
      );
      await tester.pumpAndSettle(); // border changes are animated
      expect(getBorder(tester), errorBorder);

      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isFocused: false (default)
          decoration: const InputDecoration(
            errorText: 'error',
            enabled: false,
            errorBorder: errorBorder,
            focusedBorder: focusedBorder,
            focusedErrorBorder: focusedErrorBorder,
            disabledBorder: disabledBorder,
            enabledBorder: enabledBorder,
          ),
        ),
      );
      await tester.pumpAndSettle(); // border changes are animated
      expect(getBorder(tester), errorBorder);

      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isFocused: false (default)
          decoration: const InputDecoration(
            enabled: false,
            errorBorder: errorBorder,
            focusedBorder: focusedBorder,
            focusedErrorBorder: focusedErrorBorder,
            disabledBorder: disabledBorder,
            enabledBorder: enabledBorder,
          ),
        ),
      );
      await tester.pumpAndSettle(); // border changes are animated
      expect(getBorder(tester), disabledBorder);

      await tester.pumpWidget(
        buildInputDecoratorM2(
          isFocused: true,
          decoration: const InputDecoration(
            // errorText: null (default)
            enabled: false,
            errorBorder: errorBorder,
            focusedBorder: focusedBorder,
            focusedErrorBorder: focusedErrorBorder,
            disabledBorder: disabledBorder,
            enabledBorder: enabledBorder,
          ),
        ),
      );
      await tester.pumpAndSettle(); // border changes are animated
      expect(getBorder(tester), disabledBorder);
    });

    testWidgets('OutlineInputBorder borders scale down to fit when large values are passed in', (
      WidgetTester tester,
    ) async {
      // This is a regression test for https://github.com/flutter/flutter/issues/34327
      const double largerBorderRadius = 200.0;
      const double smallerBorderRadius = 100.0;

      // Overall height for this InputDecorator is 56dps:
      //   12 - top padding
      //   12 - floating label (font size 16dps * 0.75 = 12)
      //    4 - floating label / input text gap
      //   16 - input text (font size 16dps)
      //   12 - bottom padding
      const double inputDecoratorHeight = 56.0;
      const double inputDecoratorWidth = 800.0;

      await tester.pumpWidget(
        buildInputDecoratorM2(
          decoration: const InputDecoration(
            filled: true,
            fillColor: Color(0xFF00FF00),
            labelText: 'label text',
            border: OutlineInputBorder(
              borderRadius: BorderRadius.only(
                // Intentionally large values that are larger than the InputDecorator
                topLeft: Radius.circular(smallerBorderRadius),
                bottomLeft: Radius.circular(smallerBorderRadius),
                topRight: Radius.circular(largerBorderRadius),
                bottomRight: Radius.circular(largerBorderRadius),
              ),
            ),
          ),
        ),
      );

      // Skia determines the scale based on the ratios of radii to the total
      // height or width allowed. In this case, it is the right side of the
      // border, which have two corners with largerBorderRadius that add up
      // to be 400.0.
      const double denominator = largerBorderRadius * 2.0;

      const double largerBorderRadiusScaled =
          largerBorderRadius / denominator * inputDecoratorHeight;
      const double smallerBorderRadiusScaled =
          smallerBorderRadius / denominator * inputDecoratorHeight;

      expect(
        findBorderPainter(),
        paints
          ..save()
          ..path(
            style: PaintingStyle.fill,
            color: const Color(0xFF00FF00),
            includes: const <Offset>[
              // The border should draw along the four edges of the
              // InputDecorator.

              // Top center
              Offset(inputDecoratorWidth / 2.0, 0.0),
              // Bottom center
              Offset(inputDecoratorWidth / 2.0, inputDecoratorHeight),
              // Left center
              Offset(0.0, inputDecoratorHeight / 2.0),
              // Right center
              Offset(inputDecoratorWidth, inputDecoratorHeight / 2.0),

              // The border path should contain points where each rounded corner
              // ends.

              // Bottom-right arc
              Offset(inputDecoratorWidth, inputDecoratorHeight - largerBorderRadiusScaled),
              Offset(inputDecoratorWidth - largerBorderRadiusScaled, inputDecoratorHeight),
              // Top-right arc
              Offset(inputDecoratorWidth, 0.0 + largerBorderRadiusScaled),
              Offset(inputDecoratorWidth - largerBorderRadiusScaled, 0.0),
              // Bottom-left arc
              Offset(0.0, inputDecoratorHeight - smallerBorderRadiusScaled),
              Offset(0.0 + smallerBorderRadiusScaled, inputDecoratorHeight),
              // Top-left arc
              Offset(0.0, 0.0 + smallerBorderRadiusScaled),
              Offset(0.0 + smallerBorderRadiusScaled, 0.0),
            ],
            excludes: const <Offset>[
              // The border should not contain the corner points, since the border
              // is rounded.

              // Top-left
              Offset.zero,
              // Top-right
              Offset(inputDecoratorWidth, 0.0),
              // Bottom-left
              Offset(0.0, inputDecoratorHeight),
              // Bottom-right
              Offset(inputDecoratorWidth, inputDecoratorHeight),

              // Corners with larger border ratio should not contain points outside
              // of the larger radius.

              // Bottom-right arc
              Offset(inputDecoratorWidth, inputDecoratorHeight - smallerBorderRadiusScaled),
              Offset(inputDecoratorWidth - smallerBorderRadiusScaled, inputDecoratorWidth),
              // Top-left arc
              Offset(inputDecoratorWidth, 0.0 + smallerBorderRadiusScaled),
              Offset(inputDecoratorWidth - smallerBorderRadiusScaled, 0.0),
            ],
          )
          ..restore(),
      );
    }, skip: isBrowser); // https://github.com/flutter/flutter/issues/55317

    testWidgets('rounded OutlineInputBorder with zero padding just wraps the label', (
      WidgetTester tester,
    ) async {
      // This is a regression test for https://github.com/flutter/flutter/issues/82321
      const double borderRadius = 30.0;
      const String labelText = 'label text';

      // Overall height for this InputDecorator is 56dps:
      //   12 - top padding
      //   12 - floating label (font size 16dps * 0.75 = 12)
      //    4 - floating label / input text gap
      //   16 - input text (font size 16dps)
      //   12 - bottom padding
      const double inputDecoratorHeight = 56.0;
      const double inputDecoratorWidth = 800.0;

      await tester.pumpWidget(
        buildInputDecoratorM2(
          decoration: InputDecoration(
            filled: true,
            fillColor: const Color(0xFF00FF00),
            labelText: labelText,
            border: OutlineInputBorder(
              borderRadius: BorderRadius.circular(borderRadius),
              gapPadding: 0.0,
            ),
          ),
        ),
      );

      const double denominator = borderRadius * 2.0;
      const double borderRadiusScaled = borderRadius / denominator * inputDecoratorHeight;

      expect(find.text(labelText), findsOneWidget);
      final Rect labelRect = tester.getRect(find.text(labelText));

      expect(
        findBorderPainter(),
        paints
          ..save()
          ..path(
            style: PaintingStyle.fill,
            color: const Color(0xFF00FF00),
            includes: <Offset>[
              // The border should draw along the four edges of the
              // InputDecorator.

              // Top center
              const Offset(inputDecoratorWidth / 2.0, 0.0),
              // Bottom center
              const Offset(inputDecoratorWidth / 2.0, inputDecoratorHeight),
              // Left center
              const Offset(0.0, inputDecoratorHeight / 2.0),
              // Right center
              const Offset(inputDecoratorWidth, inputDecoratorHeight / 2.0),

              // The border path should contain points where each rounded corner
              // ends.

              // Bottom-right arc
              const Offset(inputDecoratorWidth, inputDecoratorHeight - borderRadiusScaled),
              const Offset(inputDecoratorWidth - borderRadiusScaled, inputDecoratorHeight),
              // Top-right arc
              const Offset(inputDecoratorWidth, 0.0 + borderRadiusScaled),
              const Offset(inputDecoratorWidth - borderRadiusScaled, 0.0),
              // Bottom-left arc
              const Offset(0.0, inputDecoratorHeight - borderRadiusScaled),
              const Offset(0.0 + borderRadiusScaled, inputDecoratorHeight),
              // Top-left arc
              const Offset(0.0, 0.0 + borderRadiusScaled),
              const Offset(0.0 + borderRadiusScaled, 0.0),

              // Gap edges
              // gap start x = radius - radius * cos(arc sweep)
              // gap start y = radius - radius * sin(arc sweep)
              const Offset(39.49999999999999, 32.284366616798906),
              Offset(39.49999999999999 + labelRect.width, 0.0),
            ],
            excludes: const <Offset>[
              // The border should not contain the corner points, since the border
              // is rounded.

              // Top-left
              Offset.zero,
              // Top-right
              Offset(inputDecoratorWidth, 0.0),
              // Bottom-left
              Offset(0.0, inputDecoratorHeight),
              // Bottom-right
              Offset(inputDecoratorWidth, inputDecoratorHeight),
            ],
          )
          ..restore(),
      );
    }, skip: isBrowser); // https://github.com/flutter/flutter/issues/55317

    testWidgets('OutlineInputBorder with BorderRadius.zero should draw a rectangular border', (
      WidgetTester tester,
    ) async {
      // Regression test for https://github.com/flutter/flutter/issues/78855
      const String labelText = 'Flutter';

      // Overall height for this InputDecorator is 56dps:
      //   12 - top padding
      //   12 - floating label (font size 16dps * 0.75 = 12)
      //    4 - floating label / input text gap
      //   16 - input text (font size 16dps)
      //   12 - bottom padding
      const double inputDecoratorHeight = 56.0;
      const double inputDecoratorWidth = 800.0;
      const double borderWidth = 4.0;

      await tester.pumpWidget(
        buildInputDecoratorM2(
          isFocused: true,
          decoration: const InputDecoration(
            filled: false,
            labelText: labelText,
            focusedBorder: OutlineInputBorder(
              borderRadius: BorderRadius.zero,
              borderSide: BorderSide(width: borderWidth, color: Colors.red),
            ),
          ),
        ),
      );

      expect(find.text(labelText), findsOneWidget);
      expect(
        findBorderPainter(),
        paints
          ..save()
          ..path(
            includes: const <Offset>[
              // Corner points in the middle of the border line should be in the path.
              // The path is not filled and borderWidth is 4.0 so Offset(2.0, 2.0) is in the path and Offset(1.0, 1.0) is not.
              // See Skia SkPath::contains method.

              // Top-left
              Offset(borderWidth / 2, borderWidth / 2),
              // Top-right
              Offset(inputDecoratorWidth - 1 - borderWidth / 2, borderWidth / 2),
              // Bottom-left
              Offset(borderWidth / 2, inputDecoratorHeight - 1 - borderWidth / 2),
              // Bottom-right
              Offset(
                inputDecoratorWidth - 1 - borderWidth / 2,
                inputDecoratorHeight - 1 - borderWidth / 2,
              ),
            ],
            excludes: const <Offset>[
              // The path is not filled and borderWidth is 4.0 so the path should not contains the corner points.
              // See Skia SkPath::contains method.

              // Top-left
              Offset.zero,
              // // Top-right
              Offset(inputDecoratorWidth - 1, 0),
              // // Bottom-left
              Offset(0, inputDecoratorHeight - 1),
              // // Bottom-right
              Offset(inputDecoratorWidth - 1, inputDecoratorHeight - 1),
            ],
          )
          ..restore(),
      );
    }, skip: isBrowser); // https://github.com/flutter/flutter/issues/55317

    testWidgets('uses alphabetic baseline for CJK layout', (WidgetTester tester) async {
      await tester.binding.setLocale('zh', 'CN');
      final Typography typography = Typography.material2018();

      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);
      final TextEditingController controller = TextEditingController();
      addTearDown(controller.dispose);

      // The dense theme uses ideographic baselines
      Widget buildFrame(bool alignLabelWithHint) {
        return MaterialApp(
          theme: ThemeData(useMaterial3: false, textTheme: typography.dense),
          home: Material(
            child: Directionality(
              textDirection: TextDirection.ltr,
              child: TextField(
                controller: controller,
                focusNode: focusNode,
                decoration: InputDecoration(
                  labelText: 'label',
                  alignLabelWithHint: alignLabelWithHint,
                  hintText: 'hint',
                  hintStyle: const TextStyle(fontFamily: 'Cough'),
                ),
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildFrame(true));
      await tester.pumpAndSettle();

      // These numbers should be the values from using alphabetic baselines:
      // Ideographic (incorrect) value is 31.299999713897705
      expect(tester.getTopLeft(find.text('hint')).dy, 28.75);

      // Ideographic (incorrect) value is 50.299999713897705
      expect(tester.getBottomLeft(find.text('hint')).dy, isBrowser ? 45.75 : 47.75);
    });

    testWidgets('InputDecorator floating label Y coordinate', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/54028
      await tester.pumpWidget(
        buildInputDecoratorM2(
          isEmpty: true,
          decoration: const InputDecoration(
            labelText: 'label',
            enabledBorder: OutlineInputBorder(borderSide: BorderSide(width: 4)),
            floatingLabelBehavior: FloatingLabelBehavior.always,
          ),
        ),
      );

      await tester.pumpAndSettle();

      // floatingLabelHeight = 12 (font size 16dps * 0.75 = 12)
      // labelY = -floatingLabelHeight/2 + borderWidth/2
      expect(tester.getTopLeft(find.text('label')).dy, -4.0);
    });

    testWidgets('InputDecorator floating label obeys floatingLabelBehavior', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          decoration: const InputDecoration(
            labelText: 'label',
            floatingLabelBehavior: FloatingLabelBehavior.never,
          ),
        ),
      );

      // Passing floating behavior never results in a dy offset of 20
      // because the label is not initially floating.
      expect(tester.getBottomLeft(find.text('label')).dy, 36.0);
      expect(tester.getTopLeft(find.text('label')).dy, 20.0);
    });

    testWidgets('InputDecorator hint is displayed when floatingLabelBehavior is always', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isFocused: false (default)
          isEmpty: true,
          decoration: const InputDecoration(
            floatingLabelBehavior: FloatingLabelBehavior.always,
            hintText: 'hint',
            labelText: 'label',
          ),
        ),
      );
      await tester.pumpAndSettle();

      expect(getOpacity(tester, 'hint'), 1.0);
    });

    testWidgets('InputDecorator floating label width scales when focused', (
      WidgetTester tester,
    ) async {
      final String longStringA = String.fromCharCodes(List<int>.generate(200, (_) => 65));
      final String longStringB = String.fromCharCodes(List<int>.generate(200, (_) => 66));

      await tester.pumpWidget(
        Center(
          child: SizedBox(
            width: 100,
            height: 100,
            child: buildInputDecoratorM2(
              // isFocused: false (default)
              isEmpty: true,
              decoration: InputDecoration(labelText: longStringA),
            ),
          ),
        ),
      );

      await tester.pumpAndSettle();

      expect(
        find.text(longStringA),
        paints..clipRect(rect: const Rect.fromLTWH(0, 0, 100.0, 16.0)),
      );

      await tester.pumpWidget(
        Center(
          child: SizedBox(
            width: 100,
            height: 100,
            child: buildInputDecoratorM2(
              isFocused: true,
              isEmpty: true,
              decoration: InputDecoration(labelText: longStringB),
            ),
          ),
        ),
      );

      await tester.pumpAndSettle();

      expect(
        find.text(longStringB),
        paints..something((Symbol methodName, List<dynamic> arguments) {
          if (methodName != #clipRect) {
            return false;
          }
          final Rect clipRect = arguments[0] as Rect;
          // _kFinalLabelScale = 0.75
          expect(
            clipRect,
            rectMoreOrLessEquals(const Rect.fromLTWH(0, 0, 100 / 0.75, 16.0), epsilon: 1e-5),
          );
          return true;
        }),
      );
    }, skip: isBrowser); // TODO(yjbanov): https://github.com/flutter/flutter/issues/44020

    testWidgets('textAlignVertical can be updated', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/56933
      const String hintText = 'hint';
      TextAlignVertical? alignment = TextAlignVertical.top;
      late StateSetter setState;
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: StatefulBuilder(
            builder: (BuildContext context, StateSetter setter) {
              setState = setter;
              return InputDecorator(
                textAlignVertical: alignment,
                decoration: const InputDecoration(hintText: hintText),
              );
            },
          ),
        ),
      );

      final double topPosition = tester.getTopLeft(find.text(hintText)).dy;

      setState(() {
        alignment = TextAlignVertical.bottom;
      });
      await tester.pump();

      expect(tester.getTopLeft(find.text(hintText)).dy, greaterThan(topPosition));

      // Setting textAlignVertical back to null works and reverts to the default.
      setState(() {
        alignment = null;
      });
      await tester.pump();

      expect(tester.getTopLeft(find.text(hintText)).dy, topPosition);
    });

    testWidgets(
      'InputDecorationTheme floatingLabelStyle overrides label widget styles when the widget is a text widget (focused)',
      (WidgetTester tester) async {
        const TextStyle style16 = TextStyle(fontSize: 16.0);
        final TextStyle floatingLabelStyle = style16.merge(const TextStyle(color: Colors.indigo));

        // This test also verifies that the default InputDecorator provides a
        // "small concession to backwards compatibility" by not padding on
        // the left and right. If filled is true or an outline border is
        // provided then the horizontal padding is included.

        await tester.pumpWidget(
          buildInputDecoratorM2(
            isEmpty: true,
            isFocused: true, // Label appears floating above input field.
            inputDecorationTheme: InputDecorationTheme(
              floatingLabelStyle: floatingLabelStyle,
              // filled: false (default) - don't pad by left/right 12dps
            ),
            decoration: const InputDecoration(label: Text.rich(TextSpan(text: 'label'))),
          ),
        );

        // Overall height for this InputDecorator is 56dps:
        //   12 - top padding
        //   12 - floating label (font size 16dps * 0.75 = 12)
        //    4 - floating label / input text gap
        //   16 - input text (font size 16dps)
        //   12 - bottom padding
        expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
        expect(tester.getTopLeft(find.text('label')).dy, 12.0);
        expect(tester.getBottomLeft(find.text('label')).dy, 24.0);
        expect(getBorderBottom(tester), 56.0);
        expect(getBorderWeight(tester), 2.0);

        // Verify that the styles were passed along
        expect(getLabelStyle(tester).color, floatingLabelStyle.color);
      },
    );

    testWidgets(
      'InputDecorationTheme labelStyle overrides label widget styles when the widget is a text widget',
      (WidgetTester tester) async {
        const TextStyle styleDefaultSize = TextStyle(fontSize: 16.0);
        final TextStyle labelStyle = styleDefaultSize.merge(const TextStyle(color: Colors.purple));

        // This test also verifies that the default InputDecorator provides a
        // "small concession to backwards compatibility" by not padding on
        // the left and right. If filled is true or an outline border is
        // provided then the horizontal padding is included.

        await tester.pumpWidget(
          buildInputDecoratorM2(
            isEmpty: true, // Label appears inline, on top of the input field.
            inputDecorationTheme: InputDecorationTheme(
              labelStyle: labelStyle,
              // filled: false (default) - don't pad by left/right 12dps
            ),
            decoration: const InputDecoration(label: Text.rich(TextSpan(text: 'label'))),
          ),
        );

        // Overall height for this InputDecorator is 56dps:
        //   12 - top padding
        //   12 - floating label (font size 16dps * 0.75 = 12)
        //    4 - floating label / input text gap
        //   16 - input text (font size 16dps)
        //   12 - bottom padding
        expect(tester.getSize(find.byType(InputDecorator)), const Size(800.0, 56.0));
        expect(tester.getTopLeft(find.text('label')).dy, 20.0);
        expect(tester.getBottomLeft(find.text('label')).dy, 36.0);
        expect(getBorderBottom(tester), 56.0);
        expect(getBorderWeight(tester), 1.0);

        // Verify that the styles were passed along
        expect(getLabelStyle(tester).color, labelStyle.color);
      },
    );

    testWidgets('hint style overflow works', (WidgetTester tester) async {
      final String hintText = 'hint text' * 20;
      const TextStyle hintStyle = TextStyle(fontSize: 14.0, overflow: TextOverflow.fade);
      final InputDecoration decoration = InputDecoration(hintText: hintText, hintStyle: hintStyle);

      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: decoration,
        ),
      );
      await tester.pumpAndSettle();

      final Finder hintTextFinder = find.text(hintText);
      final Text hintTextWidget = tester.widget(hintTextFinder);
      expect(hintTextWidget.style!.overflow, decoration.hintStyle!.overflow);
    });

    testWidgets('prefixIcon in RTL with asymmetric padding', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/129591
      const InputDecoration decoration = InputDecoration(
        contentPadding: EdgeInsetsDirectional.only(end: 24),
        prefixIcon: Focus(child: Icon(Icons.search)),
      );

      await tester.pumpWidget(
        buildInputDecoratorM2(
          // isEmpty: false (default)
          // isFocused: false (default)
          decoration: decoration,
          textDirection: TextDirection.rtl,
        ),
      );
      await tester.pumpAndSettle();

      expect(find.byType(InputDecorator), findsOneWidget);
      expect(find.byType(Icon), findsOneWidget);

      final Offset(dx: double decoratorRight) = tester.getTopRight(find.byType(InputDecorator));
      final Offset(dx: double prefixRight) = tester.getTopRight(find.byType(Icon));

      // The prefix is inside the decorator.
      expect(decoratorRight, lessThanOrEqualTo(prefixRight));
    });

    testWidgets('intrinsic width with prefixIcon/suffixIcon', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/137937
      for (final TextDirection direction in TextDirection.values) {
        Future<Size> measureText(InputDecoration decoration) async {
          await tester.pumpWidget(
            buildInputDecoratorM2(
              // isEmpty: false (default)
              // isFocused: false (default)
              decoration: decoration,
              useIntrinsicWidth: true,
              textDirection: direction,
            ),
          );
          await tester.pumpAndSettle();

          expect(find.text('text'), findsOneWidget);

          return tester.renderObject<RenderBox>(find.text('text')).size;
        }

        const EdgeInsetsGeometry padding = EdgeInsetsDirectional.only(end: 24, start: 12);

        final Size textSizeWithoutIcons = await measureText(
          const InputDecoration(contentPadding: padding),
        );

        final Size textSizeWithPrefixIcon = await measureText(
          const InputDecoration(
            contentPadding: padding,
            prefixIcon: Focus(child: Icon(Icons.search)),
          ),
        );

        final Size textSizeWithSuffixIcon = await measureText(
          const InputDecoration(
            contentPadding: padding,
            suffixIcon: Focus(child: Icon(Icons.search)),
          ),
        );

        expect(
          textSizeWithPrefixIcon.width,
          equals(textSizeWithoutIcons.width),
          reason: 'text width is different with prefixIcon and $direction',
        );
        expect(
          textSizeWithSuffixIcon.width,
          equals(textSizeWithoutIcons.width),
          reason: 'text width is different with prefixIcon and $direction',
        );
      }
    });

    testWidgets('InputDecorator with counter does not crash when given a 0 size', (
      WidgetTester tester,
    ) async {
      // Regression test for https://github.com/flutter/flutter/issues/129611
      const InputDecoration decoration = InputDecoration(
        contentPadding: EdgeInsetsDirectional.all(99),
        prefixIcon: Focus(child: Icon(Icons.search)),
        counter: Text('COUNTER'),
      );

      await tester.pumpWidget(
        Center(
          child: SizedBox.square(
            dimension: 0.0,
            child: buildInputDecoratorM2(decoration: decoration),
          ),
        ),
      );
      await tester.pumpAndSettle();

      expect(find.byType(InputDecorator), findsOneWidget);
      expect(tester.renderObject<RenderBox>(find.text('COUNTER')).size, Size.zero);
    });
  });

  testWidgets('UnderlineInputBorder with BorderStyle.none should not show anything', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/143746
    const InputDecoration decoration = InputDecoration(
      enabledBorder: UnderlineInputBorder(
        borderSide: BorderSide(style: BorderStyle.none),
        borderRadius: BorderRadius.all(Radius.circular(4)),
      ),
    );

    await tester.pumpWidget(buildInputDecorator(decoration: decoration));
    final RenderBox box = tester.renderObject(find.byType(InputDecorator));
    expect(box, isNot(paints..drrect()));
  });
}
