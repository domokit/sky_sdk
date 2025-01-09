// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('SliderThemeData copyWith, ==, hashCode basics', () {
    expect(const SliderThemeData(), const SliderThemeData().copyWith());
    expect(const SliderThemeData().hashCode, const SliderThemeData().copyWith().hashCode);
  });

  test('SliderThemeData lerp special cases', () {
    const SliderThemeData data = SliderThemeData();
    expect(identical(SliderThemeData.lerp(data, data, 0.5), data), true);
  });

  testWidgets('Default SliderThemeData debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const SliderThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('SliderThemeData implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const SliderThemeData(
      trackHeight: 7.0,
      activeTrackColor: Color(0xFF000001),
      inactiveTrackColor: Color(0xFF000002),
      secondaryActiveTrackColor: Color(0xFF000003),
      disabledActiveTrackColor: Color(0xFF000004),
      disabledInactiveTrackColor: Color(0xFF000005),
      disabledSecondaryActiveTrackColor: Color(0xFF000006),
      activeTickMarkColor: Color(0xFF000007),
      inactiveTickMarkColor: Color(0xFF000008),
      disabledActiveTickMarkColor: Color(0xFF000009),
      disabledInactiveTickMarkColor: Color(0xFF000010),
      thumbColor: Color(0xFF000011),
      overlappingShapeStrokeColor: Color(0xFF000012),
      disabledThumbColor: Color(0xFF000013),
      overlayColor: Color(0xFF000014),
      valueIndicatorColor: Color(0xFF000015),
      valueIndicatorStrokeColor: Color(0xFF000015),
      overlayShape: RoundSliderOverlayShape(),
      tickMarkShape: RoundSliderTickMarkShape(),
      thumbShape: RoundSliderThumbShape(),
      trackShape: RoundedRectSliderTrackShape(),
      valueIndicatorShape: PaddleSliderValueIndicatorShape(),
      rangeTickMarkShape: RoundRangeSliderTickMarkShape(),
      rangeThumbShape: RoundRangeSliderThumbShape(),
      rangeTrackShape: RoundedRectRangeSliderTrackShape(),
      rangeValueIndicatorShape: PaddleRangeSliderValueIndicatorShape(),
      showValueIndicator: ShowValueIndicator.always,
      valueIndicatorTextStyle: TextStyle(color: Colors.black),
      mouseCursor: MaterialStateMouseCursor.clickable,
      allowedInteraction: SliderInteraction.tapOnly,
      padding: EdgeInsets.all(1.0),
      thumbSize: WidgetStatePropertyAll<Size>(Size(20, 20)),
      trackGap: 10.0,
      year2023: false,
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[
      'trackHeight: 7.0',
      'activeTrackColor: ${const Color(0xff000001)}',
      'inactiveTrackColor: ${const Color(0xff000002)}',
      'secondaryActiveTrackColor: ${const Color(0xff000003)}',
      'disabledActiveTrackColor: ${const Color(0xff000004)}',
      'disabledInactiveTrackColor: ${const Color(0xff000005)}',
      'disabledSecondaryActiveTrackColor: ${const Color(0xff000006)}',
      'activeTickMarkColor: ${const Color(0xff000007)}',
      'inactiveTickMarkColor: ${const Color(0xff000008)}',
      'disabledActiveTickMarkColor: ${const Color(0xff000009)}',
      'disabledInactiveTickMarkColor: ${const Color(0xff000010)}',
      'thumbColor: ${const Color(0xff000011)}',
      'overlappingShapeStrokeColor: ${const Color(0xff000012)}',
      'disabledThumbColor: ${const Color(0xff000013)}',
      'overlayColor: ${const Color(0xff000014)}',
      'valueIndicatorColor: ${const Color(0xff000015)}',
      'valueIndicatorStrokeColor: ${const Color(0xff000015)}',
      "overlayShape: Instance of 'RoundSliderOverlayShape'",
      "tickMarkShape: Instance of 'RoundSliderTickMarkShape'",
      "thumbShape: Instance of 'RoundSliderThumbShape'",
      "trackShape: Instance of 'RoundedRectSliderTrackShape'",
      "valueIndicatorShape: Instance of 'PaddleSliderValueIndicatorShape'",
      "rangeTickMarkShape: Instance of 'RoundRangeSliderTickMarkShape'",
      "rangeThumbShape: Instance of 'RoundRangeSliderThumbShape'",
      "rangeTrackShape: Instance of 'RoundedRectRangeSliderTrackShape'",
      "rangeValueIndicatorShape: Instance of 'PaddleRangeSliderValueIndicatorShape'",
      'showValueIndicator: always',
      'valueIndicatorTextStyle: TextStyle(inherit: true, color: ${const Color(0xff000000)})',
      'mouseCursor: WidgetStateMouseCursor(clickable)',
      'allowedInteraction: tapOnly',
      'padding: EdgeInsets.all(1.0)',
      'thumbSize: WidgetStatePropertyAll(Size(20.0, 20.0))',
      'trackGap: 10.0',
      'year2023: false',
    ]);
  });

  testWidgets('Slider defaults', (WidgetTester tester) async {
    debugDisableShadows = false;
    final ThemeData theme = ThemeData(useMaterial3: true);
    final ColorScheme colorScheme = theme.colorScheme;
    const double trackHeight = 4.0;
    final Color activeTrackColor = Color(colorScheme.primary.value);
    final Color inactiveTrackColor = colorScheme.surfaceContainerHighest;
    final Color secondaryActiveTrackColor = colorScheme.primary.withOpacity(0.54);
    final Color disabledActiveTrackColor = colorScheme.onSurface.withOpacity(0.38);
    final Color disabledInactiveTrackColor = colorScheme.onSurface.withOpacity(0.12);
    final Color disabledSecondaryActiveTrackColor = colorScheme.onSurface.withOpacity(0.12);
    final Color shadowColor = colorScheme.shadow;
    final Color thumbColor = Color(colorScheme.primary.value);
    final Color disabledThumbColor = Color.alphaBlend(
      colorScheme.onSurface.withOpacity(0.38),
      colorScheme.surface,
    );
    final Color activeTickMarkColor = colorScheme.onPrimary.withOpacity(0.38);
    final Color inactiveTickMarkColor = colorScheme.onSurfaceVariant.withOpacity(0.38);
    final Color disabledActiveTickMarkColor = colorScheme.onSurface.withOpacity(0.38);
    final Color disabledInactiveTickMarkColor = colorScheme.onSurface.withOpacity(0.38);

    try {
      double value = 0.45;
      Widget buildApp({int? divisions, bool enabled = true}) {
        final ValueChanged<double>? onChanged =
            !enabled
                ? null
                : (double d) {
                  value = d;
                };
        return MaterialApp(
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: Material(
              child: Center(
                child: Theme(
                  data: theme,
                  child: Slider(
                    value: value,
                    secondaryTrackValue: 0.75,
                    label: '$value',
                    divisions: divisions,
                    onChanged: onChanged,
                  ),
                ),
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp());

      final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

      // Test default track height.
      const Radius radius = Radius.circular(trackHeight / 2);
      const Radius activatedRadius = Radius.circular((trackHeight + 2) / 2);
      expect(
        material,
        paints
          // Inactive track.
          ..rrect(
            rrect: RRect.fromLTRBR(360.4, 298.0, 776.0, 302.0, radius),
            color: inactiveTrackColor,
          )
          // Active track.
          ..rrect(
            rrect: RRect.fromLTRBR(24.0, 297.0, 364.4, 303.0, activatedRadius),
            color: activeTrackColor,
          ),
      );

      // Test default colors for enabled slider.
      expect(
        material,
        paints
          ..rrect(color: inactiveTrackColor)
          ..rrect(color: activeTrackColor)
          ..rrect(color: secondaryActiveTrackColor),
      );
      expect(material, paints..shadow(color: shadowColor));
      expect(material, paints..circle(color: thumbColor));
      expect(material, isNot(paints..circle(color: disabledThumbColor)));
      expect(material, isNot(paints..rrect(color: disabledActiveTrackColor)));
      expect(material, isNot(paints..rrect(color: disabledInactiveTrackColor)));
      expect(material, isNot(paints..rrect(color: disabledSecondaryActiveTrackColor)));
      expect(material, isNot(paints..circle(color: activeTickMarkColor)));
      expect(material, isNot(paints..circle(color: inactiveTickMarkColor)));

      // Test defaults colors for discrete slider.
      await tester.pumpWidget(buildApp(divisions: 3));
      expect(
        material,
        paints
          ..rrect(color: inactiveTrackColor)
          ..rrect(color: activeTrackColor)
          ..rrect(color: secondaryActiveTrackColor),
      );
      expect(
        material,
        paints
          ..circle(color: activeTickMarkColor)
          ..circle(color: activeTickMarkColor)
          ..circle(color: inactiveTickMarkColor)
          ..circle(color: inactiveTickMarkColor)
          ..shadow(color: Colors.black)
          ..circle(color: thumbColor),
      );
      expect(material, isNot(paints..circle(color: disabledThumbColor)));
      expect(material, isNot(paints..rrect(color: disabledActiveTrackColor)));
      expect(material, isNot(paints..rrect(color: disabledInactiveTrackColor)));
      expect(material, isNot(paints..rrect(color: disabledSecondaryActiveTrackColor)));

      // Test defaults colors for disabled slider.
      await tester.pumpWidget(buildApp(enabled: false));
      await tester.pumpAndSettle();
      expect(
        material,
        paints
          ..rrect(color: disabledInactiveTrackColor)
          ..rrect(color: disabledActiveTrackColor)
          ..rrect(color: disabledSecondaryActiveTrackColor),
      );
      expect(
        material,
        paints
          ..shadow(color: shadowColor)
          ..circle(color: disabledThumbColor),
      );
      expect(material, isNot(paints..circle(color: thumbColor)));
      expect(material, isNot(paints..rrect(color: activeTrackColor)));
      expect(material, isNot(paints..rrect(color: inactiveTrackColor)));
      expect(material, isNot(paints..rrect(color: secondaryActiveTrackColor)));

      // Test defaults colors for disabled discrete slider.
      await tester.pumpWidget(buildApp(divisions: 3, enabled: false));
      expect(
        material,
        paints
          ..circle(color: disabledActiveTickMarkColor)
          ..circle(color: disabledActiveTickMarkColor)
          ..circle(color: disabledInactiveTickMarkColor)
          ..circle(color: disabledInactiveTickMarkColor)
          ..shadow(color: shadowColor)
          ..circle(color: disabledThumbColor),
      );
      expect(material, isNot(paints..circle(color: thumbColor)));
      expect(material, isNot(paints..rrect(color: activeTrackColor)));
      expect(material, isNot(paints..rrect(color: inactiveTrackColor)));
      expect(material, isNot(paints..rrect(color: secondaryActiveTrackColor)));
    } finally {
      debugDisableShadows = true;
    }
  });

  testWidgets('Slider uses the right theme colors for the right components', (
    WidgetTester tester,
  ) async {
    debugDisableShadows = false;
    try {
      const Color customColor1 = Color(0xcafefeed);
      const Color customColor2 = Color(0xdeadbeef);
      const Color customColor3 = Color(0xdecaface);
      final ThemeData theme = ThemeData(
        useMaterial3: false,
        platform: TargetPlatform.android,
        primarySwatch: Colors.blue,
        sliderTheme: const SliderThemeData(
          disabledThumbColor: Color(0xff000001),
          disabledActiveTickMarkColor: Color(0xff000002),
          disabledActiveTrackColor: Color(0xff000003),
          disabledInactiveTickMarkColor: Color(0xff000004),
          disabledInactiveTrackColor: Color(0xff000005),
          activeTrackColor: Color(0xff000006),
          activeTickMarkColor: Color(0xff000007),
          inactiveTrackColor: Color(0xff000008),
          inactiveTickMarkColor: Color(0xff000009),
          overlayColor: Color(0xff000010),
          thumbColor: Color(0xff000011),
          valueIndicatorColor: Color(0xff000012),
          disabledSecondaryActiveTrackColor: Color(0xff000013),
          secondaryActiveTrackColor: Color(0xff000014),
        ),
      );
      final SliderThemeData sliderTheme = theme.sliderTheme;
      double value = 0.45;
      Widget buildApp({
        Color? activeColor,
        Color? inactiveColor,
        Color? secondaryActiveColor,
        int? divisions,
        bool enabled = true,
      }) {
        final ValueChanged<double>? onChanged =
            !enabled
                ? null
                : (double d) {
                  value = d;
                };
        return MaterialApp(
          theme: theme,
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: Material(
              child: Center(
                child: Theme(
                  data: theme,
                  child: Slider(
                    value: value,
                    secondaryTrackValue: 0.75,
                    label: '$value',
                    divisions: divisions,
                    activeColor: activeColor,
                    inactiveColor: inactiveColor,
                    secondaryActiveColor: secondaryActiveColor,
                    onChanged: onChanged,
                  ),
                ),
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp());

      final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));
      final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

      // Check default theme for enabled widget.
      expect(
        material,
        paints
          ..rrect(color: sliderTheme.inactiveTrackColor)
          ..rrect(color: sliderTheme.activeTrackColor)
          ..rrect(color: sliderTheme.secondaryActiveTrackColor),
      );
      expect(material, paints..shadow(color: const Color(0xff000000)));
      expect(material, paints..circle(color: sliderTheme.thumbColor));
      expect(material, isNot(paints..circle(color: sliderTheme.disabledThumbColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.disabledActiveTrackColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.disabledInactiveTrackColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.disabledSecondaryActiveTrackColor)));
      expect(material, isNot(paints..circle(color: sliderTheme.activeTickMarkColor)));
      expect(material, isNot(paints..circle(color: sliderTheme.inactiveTickMarkColor)));

      // Test setting only the activeColor.
      await tester.pumpWidget(buildApp(activeColor: customColor1));
      expect(
        material,
        paints
          ..rrect(color: sliderTheme.inactiveTrackColor)
          ..rrect(color: customColor1)
          ..rrect(color: sliderTheme.secondaryActiveTrackColor),
      );
      expect(material, paints..shadow(color: Colors.black));
      expect(material, paints..circle(color: customColor1));
      expect(material, isNot(paints..circle(color: sliderTheme.thumbColor)));
      expect(material, isNot(paints..circle(color: sliderTheme.disabledThumbColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.disabledActiveTrackColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.disabledInactiveTrackColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.disabledSecondaryActiveTrackColor)));

      // Test setting only the inactiveColor.
      await tester.pumpWidget(buildApp(inactiveColor: customColor1));
      expect(
        material,
        paints
          ..rrect(color: customColor1)
          ..rrect(color: sliderTheme.activeTrackColor)
          ..rrect(color: sliderTheme.secondaryActiveTrackColor),
      );
      expect(material, paints..shadow(color: Colors.black));
      expect(material, paints..circle(color: sliderTheme.thumbColor));
      expect(material, isNot(paints..circle(color: sliderTheme.disabledThumbColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.disabledActiveTrackColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.disabledInactiveTrackColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.disabledSecondaryActiveTrackColor)));

      // Test setting only the secondaryActiveColor.
      await tester.pumpWidget(buildApp(secondaryActiveColor: customColor1));
      expect(
        material,
        paints
          ..rrect(color: sliderTheme.inactiveTrackColor)
          ..rrect(color: sliderTheme.activeTrackColor)
          ..rrect(color: customColor1),
      );
      expect(material, paints..shadow(color: Colors.black));
      expect(material, paints..circle(color: sliderTheme.thumbColor));
      expect(material, isNot(paints..circle(color: sliderTheme.disabledThumbColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.disabledActiveTrackColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.disabledInactiveTrackColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.disabledSecondaryActiveTrackColor)));

      // Test setting both activeColor, inactiveColor, and secondaryActiveColor.
      await tester.pumpWidget(
        buildApp(
          activeColor: customColor1,
          inactiveColor: customColor2,
          secondaryActiveColor: customColor3,
        ),
      );
      expect(
        material,
        paints
          ..rrect(color: customColor2)
          ..rrect(color: customColor1)
          ..rrect(color: customColor3),
      );
      expect(material, paints..shadow(color: Colors.black));
      expect(material, paints..circle(color: customColor1));
      expect(material, isNot(paints..circle(color: sliderTheme.thumbColor)));
      expect(material, isNot(paints..circle(color: sliderTheme.disabledThumbColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.disabledActiveTrackColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.disabledInactiveTrackColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.disabledSecondaryActiveTrackColor)));

      // Test colors for discrete slider.
      await tester.pumpWidget(buildApp(divisions: 3));
      expect(
        material,
        paints
          ..rrect(color: sliderTheme.inactiveTrackColor)
          ..rrect(color: sliderTheme.activeTrackColor)
          ..rrect(color: sliderTheme.secondaryActiveTrackColor),
      );
      expect(
        material,
        paints
          ..circle(color: sliderTheme.activeTickMarkColor)
          ..circle(color: sliderTheme.activeTickMarkColor)
          ..circle(color: sliderTheme.inactiveTickMarkColor)
          ..circle(color: sliderTheme.inactiveTickMarkColor)
          ..shadow(color: Colors.black)
          ..circle(color: sliderTheme.thumbColor),
      );
      expect(material, isNot(paints..circle(color: sliderTheme.disabledThumbColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.disabledActiveTrackColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.disabledInactiveTrackColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.disabledSecondaryActiveTrackColor)));

      // Test colors for discrete slider with inactiveColor and activeColor set.
      await tester.pumpWidget(
        buildApp(
          activeColor: customColor1,
          inactiveColor: customColor2,
          secondaryActiveColor: customColor3,
          divisions: 3,
        ),
      );
      expect(
        material,
        paints
          ..rrect(color: customColor2)
          ..rrect(color: customColor1)
          ..rrect(color: customColor3),
      );
      expect(
        material,
        paints
          ..circle(color: customColor2)
          ..circle(color: customColor2)
          ..circle(color: customColor1)
          ..circle(color: customColor1)
          ..shadow(color: Colors.black)
          ..circle(color: customColor1),
      );
      expect(material, isNot(paints..circle(color: sliderTheme.thumbColor)));
      expect(material, isNot(paints..circle(color: sliderTheme.disabledThumbColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.disabledActiveTrackColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.disabledInactiveTrackColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.disabledSecondaryActiveTrackColor)));
      expect(material, isNot(paints..circle(color: sliderTheme.activeTickMarkColor)));
      expect(material, isNot(paints..circle(color: sliderTheme.inactiveTickMarkColor)));

      // Test default theme for disabled widget.
      await tester.pumpWidget(buildApp(enabled: false));
      await tester.pumpAndSettle();
      expect(
        material,
        paints
          ..rrect(color: sliderTheme.disabledInactiveTrackColor)
          ..rrect(color: sliderTheme.disabledActiveTrackColor)
          ..rrect(color: sliderTheme.disabledSecondaryActiveTrackColor),
      );
      expect(
        material,
        paints
          ..shadow(color: Colors.black)
          ..circle(color: sliderTheme.disabledThumbColor),
      );
      expect(material, isNot(paints..circle(color: sliderTheme.thumbColor)));
      // These 2 colors are too close to distinguish.
      // expect(material, isNot(paints..rrect(color: sliderTheme.activeTrackColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.inactiveTrackColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.secondaryActiveTrackColor)));

      // Test default theme for disabled discrete widget.
      await tester.pumpWidget(buildApp(divisions: 3, enabled: false));
      expect(
        material,
        paints
          ..circle(color: sliderTheme.disabledActiveTickMarkColor)
          ..circle(color: sliderTheme.disabledActiveTickMarkColor)
          ..circle(color: sliderTheme.disabledInactiveTickMarkColor)
          ..circle(color: sliderTheme.disabledInactiveTickMarkColor)
          ..shadow(color: Colors.black)
          ..circle(color: sliderTheme.disabledThumbColor),
      );
      expect(material, isNot(paints..circle(color: sliderTheme.thumbColor)));
      // These 2 colors are too close to distinguish.
      // expect(material, isNot(paints..rrect(color: sliderTheme.activeTrackColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.inactiveTrackColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.secondaryActiveTrackColor)));
      expect(material, isNot(paints..circle(color: sliderTheme.activeTickMarkColor)));
      expect(material, isNot(paints..circle(color: sliderTheme.inactiveTickMarkColor)));

      // Test setting the activeColor, inactiveColor and secondaryActiveColor for disabled widget.
      await tester.pumpWidget(
        buildApp(
          activeColor: customColor1,
          inactiveColor: customColor2,
          secondaryActiveColor: customColor3,
          enabled: false,
        ),
      );
      expect(
        material,
        paints
          ..rrect(color: sliderTheme.disabledInactiveTrackColor)
          ..rrect(color: sliderTheme.disabledActiveTrackColor)
          ..rrect(color: sliderTheme.disabledSecondaryActiveTrackColor),
      );
      expect(material, paints..circle(color: sliderTheme.disabledThumbColor));
      expect(material, isNot(paints..circle(color: sliderTheme.thumbColor)));
      // These colors are too close to distinguish.
      // expect(material, isNot(paints..rrect(color: sliderTheme.activeTrackColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.inactiveTrackColor)));
      expect(material, isNot(paints..rrect(color: sliderTheme.secondaryActiveTrackColor)));

      // Test that the default value indicator has the right colors.
      await tester.pumpWidget(buildApp(divisions: 3));
      Offset center = tester.getCenter(find.byType(Slider));
      TestGesture gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();
      expect(value, equals(2.0 / 3.0));
      expect(
        valueIndicatorBox,
        paints
          ..path(color: sliderTheme.valueIndicatorColor)
          ..paragraph(),
      );
      await gesture.up();
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();

      // Testing the custom colors are used for the indicator.
      await tester.pumpWidget(
        buildApp(divisions: 3, activeColor: customColor1, inactiveColor: customColor2),
      );
      center = tester.getCenter(find.byType(Slider));
      gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();
      expect(value, equals(2.0 / 3.0));
      expect(
        valueIndicatorBox,
        paints
          ..rrect(color: const Color(0xfffafafa))
          ..rrect(color: customColor2) // Inactive track
          ..rrect(color: customColor1) // Active track
          ..circle(color: customColor1.withOpacity(0.12)) // overlay
          ..circle(color: customColor2) // 1st tick mark
          ..circle(color: customColor2) // 2nd tick mark
          ..circle(color: customColor2) // 3rd tick mark
          ..circle(color: customColor1) // 4th tick mark
          ..shadow(color: Colors.black)
          ..circle(color: customColor1) // thumb
          ..path(color: sliderTheme.valueIndicatorColor), // indicator
      );
      await gesture.up();
    } finally {
      debugDisableShadows = true;
    }
  });

  testWidgets('Slider parameters overrides theme properties', (WidgetTester tester) async {
    debugDisableShadows = false;
    const Color activeTrackColor = Color(0xffff0001);
    const Color inactiveTrackColor = Color(0xffff0002);
    const Color secondaryActiveTrackColor = Color(0xffff0003);
    const Color thumbColor = Color(0xffff0004);

    final ThemeData theme = ThemeData(
      platform: TargetPlatform.android,
      primarySwatch: Colors.blue,
      sliderTheme: const SliderThemeData(
        activeTrackColor: Color(0xff000001),
        inactiveTickMarkColor: Color(0xff000002),
        secondaryActiveTrackColor: Color(0xff000003),
        thumbColor: Color(0xff000004),
      ),
    );
    try {
      const double value = 0.45;
      Widget buildApp({bool enabled = true}) {
        return MaterialApp(
          theme: theme,
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: Material(
              child: Center(
                child: Slider(
                  activeColor: activeTrackColor,
                  inactiveColor: inactiveTrackColor,
                  secondaryActiveColor: secondaryActiveTrackColor,
                  thumbColor: thumbColor,
                  value: value,
                  secondaryTrackValue: 0.75,
                  label: '$value',
                  onChanged: (double value) {},
                ),
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp());

      final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

      // Test Slider parameters.
      expect(
        material,
        paints
          ..rrect(color: inactiveTrackColor)
          ..rrect(color: activeTrackColor)
          ..rrect(color: secondaryActiveTrackColor),
      );
      expect(material, paints..circle(color: thumbColor));
    } finally {
      debugDisableShadows = true;
    }
  });

  testWidgets('Slider uses ThemeData slider theme if present', (WidgetTester tester) async {
    final ThemeData theme = ThemeData(platform: TargetPlatform.android, primarySwatch: Colors.red);
    final SliderThemeData sliderTheme = theme.sliderTheme;
    final SliderThemeData customTheme = sliderTheme.copyWith(
      activeTrackColor: Colors.purple,
      inactiveTrackColor: Colors.purple.withAlpha(0x3d),
      secondaryActiveTrackColor: Colors.purple.withAlpha(0x8a),
    );

    await tester.pumpWidget(
      _buildApp(sliderTheme, value: 0.5, secondaryTrackValue: 0.75, enabled: false),
    );
    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

    expect(
      material,
      paints
        ..rrect(color: customTheme.disabledActiveTrackColor)
        ..rrect(color: customTheme.disabledInactiveTrackColor)
        ..rrect(color: customTheme.disabledSecondaryActiveTrackColor),
    );
  });

  testWidgets('Slider overrides ThemeData theme if SliderTheme present', (
    WidgetTester tester,
  ) async {
    final ThemeData theme = ThemeData(platform: TargetPlatform.android, primarySwatch: Colors.red);
    final SliderThemeData sliderTheme = theme.sliderTheme;
    final SliderThemeData customTheme = sliderTheme.copyWith(
      activeTrackColor: Colors.purple,
      inactiveTrackColor: Colors.purple.withAlpha(0x3d),
      secondaryActiveTrackColor: Colors.purple.withAlpha(0x8a),
    );

    await tester.pumpWidget(
      _buildApp(sliderTheme, value: 0.5, secondaryTrackValue: 0.75, enabled: false),
    );
    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

    expect(
      material,
      paints
        ..rrect(color: customTheme.disabledActiveTrackColor)
        ..rrect(color: customTheme.disabledInactiveTrackColor)
        ..rrect(color: customTheme.disabledSecondaryActiveTrackColor),
    );
  });

  testWidgets('SliderThemeData generates correct opacities for fromPrimaryColors', (
    WidgetTester tester,
  ) async {
    const Color customColor1 = Color(0xcafefeed);
    const Color customColor2 = Color(0xdeadbeef);
    const Color customColor3 = Color(0xdecaface);
    const Color customColor4 = Color(0xfeedcafe);

    final SliderThemeData sliderTheme = SliderThemeData.fromPrimaryColors(
      primaryColor: customColor1,
      primaryColorDark: customColor2,
      primaryColorLight: customColor3,
      valueIndicatorTextStyle: ThemeData.fallback().textTheme.bodyLarge!.copyWith(
        color: customColor4,
      ),
    );

    expect(sliderTheme.activeTrackColor, equals(customColor1.withAlpha(0xff)));
    expect(sliderTheme.inactiveTrackColor, equals(customColor1.withAlpha(0x3d)));
    expect(sliderTheme.secondaryActiveTrackColor, equals(customColor1.withAlpha(0x8a)));
    expect(sliderTheme.disabledActiveTrackColor, equals(customColor2.withAlpha(0x52)));
    expect(sliderTheme.disabledInactiveTrackColor, equals(customColor2.withAlpha(0x1f)));
    expect(sliderTheme.disabledSecondaryActiveTrackColor, equals(customColor2.withAlpha(0x1f)));
    expect(sliderTheme.activeTickMarkColor, equals(customColor3.withAlpha(0x8a)));
    expect(sliderTheme.inactiveTickMarkColor, equals(customColor1.withAlpha(0x8a)));
    expect(sliderTheme.disabledActiveTickMarkColor, equals(customColor3.withAlpha(0x1f)));
    expect(sliderTheme.disabledInactiveTickMarkColor, equals(customColor2.withAlpha(0x1f)));
    expect(sliderTheme.thumbColor, equals(customColor1.withAlpha(0xff)));
    expect(sliderTheme.disabledThumbColor, equals(customColor2.withAlpha(0x52)));
    expect(sliderTheme.overlayColor, equals(customColor1.withAlpha(0x1f)));
    expect(sliderTheme.valueIndicatorColor, equals(customColor1.withAlpha(0xff)));
    expect(sliderTheme.valueIndicatorStrokeColor, equals(customColor1.withAlpha(0xff)));
    expect(sliderTheme.valueIndicatorTextStyle!.color, equals(customColor4));
  });

  testWidgets('SliderThemeData generates correct shapes for fromPrimaryColors', (
    WidgetTester tester,
  ) async {
    const Color customColor1 = Color(0xcafefeed);
    const Color customColor2 = Color(0xdeadbeef);
    const Color customColor3 = Color(0xdecaface);
    const Color customColor4 = Color(0xfeedcafe);

    final SliderThemeData sliderTheme = SliderThemeData.fromPrimaryColors(
      primaryColor: customColor1,
      primaryColorDark: customColor2,
      primaryColorLight: customColor3,
      valueIndicatorTextStyle: ThemeData.fallback().textTheme.bodyLarge!.copyWith(
        color: customColor4,
      ),
    );

    expect(sliderTheme.overlayShape, const RoundSliderOverlayShape());
    expect(sliderTheme.tickMarkShape, const RoundSliderTickMarkShape());
    expect(sliderTheme.thumbShape, const RoundSliderThumbShape());
    expect(sliderTheme.trackShape, const RoundedRectSliderTrackShape());
    expect(sliderTheme.valueIndicatorShape, const PaddleSliderValueIndicatorShape());
    expect(sliderTheme.rangeTickMarkShape, const RoundRangeSliderTickMarkShape());
    expect(sliderTheme.rangeThumbShape, const RoundRangeSliderThumbShape());
    expect(sliderTheme.rangeTrackShape, const RoundedRectRangeSliderTrackShape());
    expect(sliderTheme.rangeValueIndicatorShape, const PaddleRangeSliderValueIndicatorShape());
  });

  testWidgets('SliderThemeData lerps correctly', (WidgetTester tester) async {
    final SliderThemeData sliderThemeBlack = SliderThemeData.fromPrimaryColors(
      primaryColor: Colors.black,
      primaryColorDark: Colors.black,
      primaryColorLight: Colors.black,
      valueIndicatorTextStyle: ThemeData.fallback().textTheme.bodyLarge!.copyWith(
        color: Colors.black,
      ),
    ).copyWith(trackHeight: 2.0);
    final SliderThemeData sliderThemeWhite = SliderThemeData.fromPrimaryColors(
      primaryColor: Colors.white,
      primaryColorDark: Colors.white,
      primaryColorLight: Colors.white,
      valueIndicatorTextStyle: ThemeData.fallback().textTheme.bodyLarge!.copyWith(
        color: Colors.white,
      ),
    ).copyWith(trackHeight: 6.0);
    final SliderThemeData lerp = SliderThemeData.lerp(sliderThemeBlack, sliderThemeWhite, 0.5);
    const Color middleGrey = Color(0xff7f7f7f);

    expect(lerp.trackHeight, equals(4.0));
    expect(lerp.activeTrackColor, isSameColorAs(middleGrey.withAlpha(0xff)));
    expect(lerp.inactiveTrackColor, isSameColorAs(middleGrey.withAlpha(0x3d)));
    expect(lerp.secondaryActiveTrackColor, isSameColorAs(middleGrey.withAlpha(0x8a)));
    expect(lerp.disabledActiveTrackColor, isSameColorAs(middleGrey.withAlpha(0x52)));
    expect(lerp.disabledInactiveTrackColor, isSameColorAs(middleGrey.withAlpha(0x1f)));
    expect(lerp.disabledSecondaryActiveTrackColor, isSameColorAs(middleGrey.withAlpha(0x1f)));
    expect(lerp.activeTickMarkColor, isSameColorAs(middleGrey.withAlpha(0x8a)));
    expect(lerp.inactiveTickMarkColor, isSameColorAs(middleGrey.withAlpha(0x8a)));
    expect(lerp.disabledActiveTickMarkColor, isSameColorAs(middleGrey.withAlpha(0x1f)));
    expect(lerp.disabledInactiveTickMarkColor, isSameColorAs(middleGrey.withAlpha(0x1f)));
    expect(lerp.thumbColor, isSameColorAs(middleGrey.withAlpha(0xff)));
    expect(lerp.disabledThumbColor, isSameColorAs(middleGrey.withAlpha(0x52)));
    expect(lerp.overlayColor, isSameColorAs(middleGrey.withAlpha(0x1f)));
    expect(lerp.valueIndicatorColor, isSameColorAs(middleGrey.withAlpha(0xff)));
    expect(lerp.valueIndicatorStrokeColor, isSameColorAs(middleGrey.withAlpha(0xff)));
    expect(lerp.valueIndicatorTextStyle!.color, isSameColorAs(middleGrey.withAlpha(0xff)));
  });

  testWidgets('Default slider track draws correctly', (WidgetTester tester) async {
    final ThemeData theme = ThemeData(platform: TargetPlatform.android, primarySwatch: Colors.blue);
    final SliderThemeData sliderTheme = theme.sliderTheme.copyWith(thumbColor: Colors.red.shade500);

    await tester.pumpWidget(_buildApp(sliderTheme, value: 0.25, secondaryTrackValue: 0.5));
    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

    const Radius radius = Radius.circular(2);
    const Radius activatedRadius = Radius.circular(3);

    // The enabled slider thumb has track segments that extend to and from
    // the center of the thumb.
    expect(
      material,
      paints
        // Inactive track.
        ..rrect(
          rrect: RRect.fromLTRBR(210.0, 298.0, 776.0, 302.0, radius),
          color: sliderTheme.inactiveTrackColor,
        )
        // Active track.
        ..rrect(
          rrect: RRect.fromLTRBR(24.0, 297.0, 214.0, 303.0, activatedRadius),
          color: sliderTheme.activeTrackColor,
        )
        ..rrect(
          rrect: RRect.fromLTRBAndCorners(
            212.0,
            298.0,
            400.0,
            302.0,
            topRight: radius,
            bottomRight: radius,
          ),
          color: sliderTheme.secondaryActiveTrackColor,
        ),
    );

    await tester.pumpWidget(
      _buildApp(sliderTheme, value: 0.25, secondaryTrackValue: 0.5, enabled: false),
    );
    await tester.pumpAndSettle(); // wait for disable animation

    // The disabled slider thumb is the same size as the enabled thumb.
    expect(
      material,
      paints
        // Inactive track.
        ..rrect(
          rrect: RRect.fromLTRBR(210.0, 298.0, 776.0, 302.0, radius),
          color: sliderTheme.disabledInactiveTrackColor,
        )
        // Active track.
        ..rrect(
          rrect: RRect.fromLTRBR(24.0, 297.0, 214.0, 303.0, activatedRadius),
          color: sliderTheme.disabledActiveTrackColor,
        )
        ..rrect(
          rrect: RRect.fromLTRBAndCorners(
            212.0,
            298.0,
            400.0,
            302.0,
            topRight: radius,
            bottomRight: radius,
          ),
          color: sliderTheme.disabledSecondaryActiveTrackColor,
        ),
    );
  });

  testWidgets('Default slider overlay draws correctly', (WidgetTester tester) async {
    final ThemeData theme = ThemeData(platform: TargetPlatform.android, primarySwatch: Colors.blue);
    final SliderThemeData sliderTheme = theme.sliderTheme.copyWith(thumbColor: Colors.red.shade500);

    await tester.pumpWidget(_buildApp(sliderTheme, value: 0.25));
    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

    // With no touch, paints only the thumb.
    expect(
      material,
      paints..circle(color: sliderTheme.thumbColor, x: 212.0, y: 300.0, radius: 10.0),
    );

    final Offset center = tester.getCenter(find.byType(Slider));
    final TestGesture gesture = await tester.startGesture(center);
    // Wait for overlay animation to finish.
    await tester.pumpAndSettle();

    // After touch, paints thumb and overlay.
    expect(
      material,
      paints
        ..circle(color: sliderTheme.overlayColor, x: 212.0, y: 300.0, radius: 24.0)
        ..circle(color: sliderTheme.thumbColor, x: 212.0, y: 300.0, radius: 10.0),
    );

    await gesture.up();
    await tester.pumpAndSettle();

    // After the gesture is up and complete, it again paints only the thumb.
    expect(
      material,
      paints..circle(color: sliderTheme.thumbColor, x: 212.0, y: 300.0, radius: 10.0),
    );
  });

  testWidgets('Slider can use theme overlay with material states', (WidgetTester tester) async {
    final ThemeData theme = ThemeData(platform: TargetPlatform.android, primarySwatch: Colors.blue);
    final SliderThemeData sliderTheme = theme.sliderTheme.copyWith(
      overlayColor: MaterialStateColor.resolveWith((Set<MaterialState> states) {
        if (states.contains(MaterialState.focused)) {
          return Colors.brown[500]!;
        }

        return Colors.transparent;
      }),
    );
    final FocusNode focusNode = FocusNode(debugLabel: 'Slider');
    addTearDown(focusNode.dispose);
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    double value = 0.5;

    Widget buildApp({bool enabled = true}) {
      return MaterialApp(
        theme: ThemeData(sliderTheme: sliderTheme),
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Slider(
                  value: value,
                  onChanged:
                      enabled
                          ? (double newValue) {
                            setState(() {
                              value = newValue;
                            });
                          }
                          : null,
                  autofocus: true,
                  focusNode: focusNode,
                );
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp());

    // Check that the overlay shows when focused.
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      Material.of(tester.element(find.byType(Slider))),
      paints..circle(color: Colors.brown[500]),
    );

    // Check that the overlay does not show when focused and disabled.
    await tester.pumpWidget(buildApp(enabled: false));
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isFalse);
    expect(
      Material.of(tester.element(find.byType(Slider))),
      isNot(paints..circle(color: Colors.brown[500])),
    );
  });

  testWidgets('Default slider ticker and thumb shape draw correctly', (WidgetTester tester) async {
    final ThemeData theme = ThemeData(platform: TargetPlatform.android, primarySwatch: Colors.blue);
    final SliderThemeData sliderTheme = theme.sliderTheme.copyWith(thumbColor: Colors.red.shade500);

    await tester.pumpWidget(_buildApp(sliderTheme, value: 0.45));
    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

    expect(material, paints..circle(color: sliderTheme.thumbColor, radius: 10.0));

    await tester.pumpWidget(_buildApp(sliderTheme, value: 0.45, enabled: false));
    await tester.pumpAndSettle(); // wait for disable animation

    expect(material, paints..circle(color: sliderTheme.disabledThumbColor, radius: 10.0));

    await tester.pumpWidget(_buildApp(sliderTheme, value: 0.45, divisions: 3));
    await tester.pumpAndSettle(); // wait for enable animation

    expect(
      material,
      paints
        ..circle(color: sliderTheme.activeTickMarkColor)
        ..circle(color: sliderTheme.activeTickMarkColor)
        ..circle(color: sliderTheme.inactiveTickMarkColor)
        ..circle(color: sliderTheme.inactiveTickMarkColor)
        ..circle(color: sliderTheme.thumbColor, radius: 10.0),
    );

    await tester.pumpWidget(_buildApp(sliderTheme, value: 0.45, divisions: 3, enabled: false));
    await tester.pumpAndSettle(); // wait for disable animation

    expect(
      material,
      paints
        ..circle(color: sliderTheme.disabledActiveTickMarkColor)
        ..circle(color: sliderTheme.disabledInactiveTickMarkColor)
        ..circle(color: sliderTheme.disabledInactiveTickMarkColor)
        ..circle(color: sliderTheme.disabledInactiveTickMarkColor)
        ..circle(color: sliderTheme.disabledThumbColor, radius: 10.0),
    );
  });

  testWidgets('Default paddle slider value indicator shape draws correctly', (
    WidgetTester tester,
  ) async {
    debugDisableShadows = false;
    try {
      final ThemeData theme = ThemeData(
        useMaterial3: false,
        platform: TargetPlatform.android,
        primarySwatch: Colors.blue,
      );
      final SliderThemeData sliderTheme = theme.sliderTheme.copyWith(
        thumbColor: Colors.red.shade500,
        showValueIndicator: ShowValueIndicator.always,
        valueIndicatorShape: const PaddleSliderValueIndicatorShape(),
      );
      Widget buildApp(
        String value, {
        double sliderValue = 0.5,
        TextScaler textScaler = TextScaler.noScaling,
      }) {
        return MaterialApp(
          theme: theme,
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: MediaQuery(
              data: MediaQueryData(textScaler: textScaler),
              child: Material(
                child: Row(
                  children: <Widget>[
                    Expanded(
                      child: SliderTheme(
                        data: sliderTheme,
                        child: Slider(
                          value: sliderValue,
                          label: value,
                          divisions: 3,
                          onChanged: (double d) {},
                        ),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp('1'));

      final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

      Offset center = tester.getCenter(find.byType(Slider));
      TestGesture gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();
      expect(
        valueIndicatorBox,
        paints..path(
          color: sliderTheme.valueIndicatorColor,
          includes: <Offset>[
            const Offset(0.0, -40.0),
            const Offset(15.9, -40.0),
            const Offset(-15.9, -40.0),
          ],
          excludes: <Offset>[const Offset(16.1, -40.0), const Offset(-16.1, -40.0)],
        ),
      );

      await gesture.up();

      // Test that it expands with a larger label.
      await tester.pumpWidget(buildApp('1000'));
      center = tester.getCenter(find.byType(Slider));
      gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();
      expect(
        valueIndicatorBox,
        paints..path(
          color: sliderTheme.valueIndicatorColor,
          includes: <Offset>[
            const Offset(0.0, -40.0),
            const Offset(35.9, -40.0),
            const Offset(-35.9, -40.0),
          ],
          excludes: <Offset>[const Offset(36.1, -40.0), const Offset(-36.1, -40.0)],
        ),
      );
      await gesture.up();

      // Test that it avoids the left edge of the screen.
      await tester.pumpWidget(buildApp('1000000', sliderValue: 0.0));
      center = tester.getCenter(find.byType(Slider));
      gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();
      expect(
        valueIndicatorBox,
        paints..path(
          color: sliderTheme.valueIndicatorColor,
          includes: <Offset>[
            const Offset(0.0, -40.0),
            const Offset(92.0, -40.0),
            const Offset(-16.0, -40.0),
          ],
          excludes: <Offset>[const Offset(98.1, -40.0), const Offset(-20.1, -40.0)],
        ),
      );
      await gesture.up();

      // Test that it avoids the right edge of the screen.
      await tester.pumpWidget(buildApp('1000000', sliderValue: 1.0));
      center = tester.getCenter(find.byType(Slider));
      gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();
      expect(
        valueIndicatorBox,
        paints..path(
          color: sliderTheme.valueIndicatorColor,
          includes: <Offset>[
            const Offset(0.0, -40.0),
            const Offset(16.0, -40.0),
            const Offset(-92.0, -40.0),
          ],
          excludes: <Offset>[const Offset(20.1, -40.0), const Offset(-98.1, -40.0)],
        ),
      );
      await gesture.up();

      // Test that the neck stretches when the text scale gets smaller.
      await tester.pumpWidget(
        buildApp('1000000', sliderValue: 0.0, textScaler: const TextScaler.linear(0.5)),
      );
      center = tester.getCenter(find.byType(Slider));
      gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();
      expect(
        valueIndicatorBox,
        paints..path(
          color: sliderTheme.valueIndicatorColor,
          includes: <Offset>[
            const Offset(0.0, -49.0),
            const Offset(68.0, -49.0),
            const Offset(-24.0, -49.0),
          ],
          excludes: <Offset>[
            const Offset(98.0, -32.0), // inside full size, outside small
            const Offset(-40.0, -32.0), // inside full size, outside small
            const Offset(90.1, -49.0),
            const Offset(-40.1, -49.0),
          ],
        ),
      );
      await gesture.up();

      // Test that the neck shrinks when the text scale gets larger.
      await tester.pumpWidget(
        buildApp('1000000', sliderValue: 0.0, textScaler: const TextScaler.linear(2.5)),
      );
      center = tester.getCenter(find.byType(Slider));
      gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();
      expect(
        valueIndicatorBox,
        paints..path(
          color: sliderTheme.valueIndicatorColor,
          includes: <Offset>[
            const Offset(0.0, -38.8),
            const Offset(92.0, -38.8),
            const Offset(8.0, -23.0), // Inside large, outside scale=1.0
            const Offset(-2.0, -23.0), // Inside large, outside scale=1.0
          ],
          excludes: <Offset>[const Offset(98.5, -38.8), const Offset(-16.1, -38.8)],
        ),
      );
      await gesture.up();
    } finally {
      debugDisableShadows = true;
    }
  });

  testWidgets('Default paddle slider value indicator shape draws correctly', (
    WidgetTester tester,
  ) async {
    debugDisableShadows = false;
    try {
      final ThemeData theme = ThemeData(
        useMaterial3: false,
        platform: TargetPlatform.android,
        primarySwatch: Colors.blue,
      );
      final SliderThemeData sliderTheme = theme.sliderTheme.copyWith(
        thumbColor: Colors.red.shade500,
        showValueIndicator: ShowValueIndicator.always,
        valueIndicatorShape: const PaddleSliderValueIndicatorShape(),
      );
      Widget buildApp(
        String value, {
        double sliderValue = 0.5,
        TextScaler textScaler = TextScaler.noScaling,
      }) {
        return MaterialApp(
          theme: theme,
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: MediaQuery(
              data: MediaQueryData(textScaler: textScaler),
              child: Material(
                child: Row(
                  children: <Widget>[
                    Expanded(
                      child: SliderTheme(
                        data: sliderTheme,
                        child: Slider(
                          value: sliderValue,
                          label: value,
                          divisions: 3,
                          onChanged: (double d) {},
                        ),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp('1'));

      final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

      Offset center = tester.getCenter(find.byType(Slider));
      TestGesture gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();
      expect(
        valueIndicatorBox,
        paints..path(
          color: sliderTheme.valueIndicatorColor,
          includes: <Offset>[
            const Offset(0.0, -40.0),
            const Offset(15.9, -40.0),
            const Offset(-15.9, -40.0),
          ],
          excludes: <Offset>[const Offset(16.1, -40.0), const Offset(-16.1, -40.0)],
        ),
      );

      await gesture.up();

      // Test that it expands with a larger label.
      await tester.pumpWidget(buildApp('1000'));
      center = tester.getCenter(find.byType(Slider));
      gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();
      expect(
        valueIndicatorBox,
        paints..path(
          color: sliderTheme.valueIndicatorColor,
          includes: <Offset>[
            const Offset(0.0, -40.0),
            const Offset(35.9, -40.0),
            const Offset(-35.9, -40.0),
          ],
          excludes: <Offset>[const Offset(36.1, -40.0), const Offset(-36.1, -40.0)],
        ),
      );
      await gesture.up();

      // Test that it avoids the left edge of the screen.
      await tester.pumpWidget(buildApp('1000000', sliderValue: 0.0));
      center = tester.getCenter(find.byType(Slider));
      gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();
      expect(
        valueIndicatorBox,
        paints..path(
          color: sliderTheme.valueIndicatorColor,
          includes: <Offset>[
            const Offset(0.0, -40.0),
            const Offset(92.0, -40.0),
            const Offset(-16.0, -40.0),
          ],
          excludes: <Offset>[const Offset(98.1, -40.0), const Offset(-20.1, -40.0)],
        ),
      );
      await gesture.up();

      // Test that it avoids the right edge of the screen.
      await tester.pumpWidget(buildApp('1000000', sliderValue: 1.0));
      center = tester.getCenter(find.byType(Slider));
      gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();
      expect(
        valueIndicatorBox,
        paints..path(
          color: sliderTheme.valueIndicatorColor,
          includes: <Offset>[
            const Offset(0.0, -40.0),
            const Offset(16.0, -40.0),
            const Offset(-92.0, -40.0),
          ],
          excludes: <Offset>[const Offset(20.1, -40.0), const Offset(-98.1, -40.0)],
        ),
      );
      await gesture.up();

      // Test that the neck stretches when the text scale gets smaller.
      await tester.pumpWidget(
        buildApp('1000000', sliderValue: 0.0, textScaler: const TextScaler.linear(0.5)),
      );
      center = tester.getCenter(find.byType(Slider));
      gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();
      expect(
        valueIndicatorBox,
        paints..path(
          color: sliderTheme.valueIndicatorColor,
          includes: <Offset>[
            const Offset(0.0, -49.0),
            const Offset(68.0, -49.0),
            const Offset(-24.0, -49.0),
          ],
          excludes: <Offset>[
            const Offset(98.0, -32.0), // inside full size, outside small
            const Offset(-40.0, -32.0), // inside full size, outside small
            const Offset(90.1, -49.0),
            const Offset(-40.1, -49.0),
          ],
        ),
      );
      await gesture.up();

      // Test that the neck shrinks when the text scale gets larger.
      await tester.pumpWidget(
        buildApp('1000000', sliderValue: 0.0, textScaler: const TextScaler.linear(2.5)),
      );
      center = tester.getCenter(find.byType(Slider));
      gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();
      expect(
        valueIndicatorBox,
        paints..path(
          color: sliderTheme.valueIndicatorColor,
          includes: <Offset>[
            const Offset(0.0, -38.8),
            const Offset(92.0, -38.8),
            const Offset(8.0, -23.0), // Inside large, outside scale=1.0
            const Offset(-2.0, -23.0), // Inside large, outside scale=1.0
          ],
          excludes: <Offset>[const Offset(98.5, -38.8), const Offset(-16.1, -38.8)],
        ),
      );
      await gesture.up();
    } finally {
      debugDisableShadows = true;
    }
  });

  testWidgets('The slider track height can be overridden', (WidgetTester tester) async {
    final SliderThemeData sliderTheme = ThemeData().sliderTheme.copyWith(trackHeight: 16);
    const Radius radius = Radius.circular(8);
    const Radius activatedRadius = Radius.circular(9);

    await tester.pumpWidget(_buildApp(sliderTheme, value: 0.25));

    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

    // Top and bottom are centerY (300) + and - trackRadius (8).
    expect(
      material,
      paints
        // Inactive track.
        ..rrect(
          rrect: RRect.fromLTRBR(204.0, 292.0, 776.0, 308.0, radius),
          color: sliderTheme.inactiveTrackColor,
        )
        // Active track.
        ..rrect(
          rrect: RRect.fromLTRBR(24.0, 291.0, 220.0, 309.0, activatedRadius),
          color: sliderTheme.activeTrackColor,
        ),
    );

    await tester.pumpWidget(_buildApp(sliderTheme, value: 0.25, enabled: false));
    await tester.pumpAndSettle(); // wait for disable animation

    // The disabled thumb is smaller so the active track has to paint longer to
    // get to the edge.
    expect(
      material,
      paints
        // Inactive track.
        ..rrect(
          rrect: RRect.fromLTRBR(204.0, 292.0, 776.0, 308.0, radius),
          color: sliderTheme.disabledInactiveTrackColor,
        )
        // Active track.
        ..rrect(
          rrect: RRect.fromLTRBR(24.0, 291.0, 220.0, 309.0, activatedRadius),
          color: sliderTheme.disabledActiveTrackColor,
        ),
    );
  });

  testWidgets('The default slider thumb shape sizes can be overridden', (
    WidgetTester tester,
  ) async {
    final SliderThemeData sliderTheme = ThemeData().sliderTheme.copyWith(
      thumbShape: const RoundSliderThumbShape(enabledThumbRadius: 7, disabledThumbRadius: 11),
    );

    await tester.pumpWidget(_buildApp(sliderTheme, value: 0.25));
    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

    expect(material, paints..circle(x: 212, y: 300, radius: 7, color: sliderTheme.thumbColor));

    await tester.pumpWidget(_buildApp(sliderTheme, value: 0.25, enabled: false));
    await tester.pumpAndSettle(); // wait for disable animation

    expect(
      material,
      paints..circle(x: 212, y: 300, radius: 11, color: sliderTheme.disabledThumbColor),
    );
  });

  testWidgets(
    'The default slider thumb shape disabled size can be inferred from the enabled size',
    (WidgetTester tester) async {
      final SliderThemeData sliderTheme = ThemeData().sliderTheme.copyWith(
        thumbShape: const RoundSliderThumbShape(enabledThumbRadius: 9),
      );

      await tester.pumpWidget(_buildApp(sliderTheme, value: 0.25));
      final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

      expect(material, paints..circle(x: 212, y: 300, radius: 9, color: sliderTheme.thumbColor));

      await tester.pumpWidget(_buildApp(sliderTheme, value: 0.25, enabled: false));
      await tester.pumpAndSettle(); // wait for disable animation
      expect(
        material,
        paints..circle(x: 212, y: 300, radius: 9, color: sliderTheme.disabledThumbColor),
      );
    },
  );

  testWidgets('The default slider tick mark shape size can be overridden', (
    WidgetTester tester,
  ) async {
    final SliderThemeData sliderTheme = ThemeData().sliderTheme.copyWith(
      tickMarkShape: const RoundSliderTickMarkShape(tickMarkRadius: 5),
      activeTickMarkColor: const Color(0xfadedead),
      inactiveTickMarkColor: const Color(0xfadebeef),
      disabledActiveTickMarkColor: const Color(0xfadecafe),
      disabledInactiveTickMarkColor: const Color(0xfadeface),
    );

    await tester.pumpWidget(_buildApp(sliderTheme, value: 0.5, divisions: 2));

    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

    expect(
      material,
      paints
        ..circle(x: 26, y: 300, radius: 5, color: sliderTheme.activeTickMarkColor)
        ..circle(x: 400, y: 300, radius: 5, color: sliderTheme.activeTickMarkColor)
        ..circle(x: 774, y: 300, radius: 5, color: sliderTheme.inactiveTickMarkColor),
    );

    await tester.pumpWidget(_buildApp(sliderTheme, value: 0.5, divisions: 2, enabled: false));
    await tester.pumpAndSettle();

    expect(
      material,
      paints
        ..circle(x: 26, y: 300, radius: 5, color: sliderTheme.disabledActiveTickMarkColor)
        ..circle(x: 400, y: 300, radius: 5, color: sliderTheme.disabledActiveTickMarkColor)
        ..circle(x: 774, y: 300, radius: 5, color: sliderTheme.disabledInactiveTickMarkColor),
    );
  });

  testWidgets('The default slider overlay shape size can be overridden', (
    WidgetTester tester,
  ) async {
    const double uniqueOverlayRadius = 23;
    final SliderThemeData sliderTheme = ThemeData().sliderTheme.copyWith(
      overlayShape: const RoundSliderOverlayShape(overlayRadius: uniqueOverlayRadius),
    );

    await tester.pumpWidget(_buildApp(sliderTheme, value: 0.5));
    // Tap center and wait for animation.
    final Offset center = tester.getCenter(find.byType(Slider));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pumpAndSettle();

    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));
    expect(
      material,
      paints..circle(
        x: center.dx,
        y: center.dy,
        radius: uniqueOverlayRadius,
        color: sliderTheme.overlayColor,
      ),
    );

    // Finish gesture to release resources.
    await gesture.up();
    await tester.pumpAndSettle();
  });

  // Regression test for https://github.com/flutter/flutter/issues/74503
  testWidgets(
    'The slider track layout correctly when the overlay size is smaller than the thumb size',
    (WidgetTester tester) async {
      final SliderThemeData sliderTheme = ThemeData().sliderTheme.copyWith(
        overlayShape: SliderComponentShape.noOverlay,
      );

      await tester.pumpWidget(_buildApp(sliderTheme, value: 0.5));

      final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

      // The track rectangle begins at 10 pixels from the left of the screen and ends 10 pixels from the right
      // (790 pixels from the left). The main check here it that the track itself should be centered on
      // the 800 pixel-wide screen.
      expect(
        material,
        paints
          // Inactive track RRect. Ends 10 pixels from right of screen.
          ..rrect(rrect: RRect.fromLTRBR(398.0, 298.0, 790.0, 302.0, const Radius.circular(2.0)))
          // Active track RRect. Starts 10 pixels from left of screen.
          ..rrect(rrect: RRect.fromLTRBR(10.0, 297.0, 402.0, 303.0, const Radius.circular(3.0)))
          // The thumb.
          ..circle(x: 400.0, y: 300.0, radius: 10.0),
      );
    },
  );

  // Regression test for https://github.com/flutter/flutter/issues/125467
  testWidgets(
    'The RangeSlider track layout correctly when the overlay size is smaller than the thumb size',
    (WidgetTester tester) async {
      final SliderThemeData sliderTheme = ThemeData().sliderTheme.copyWith(
        overlayShape: SliderComponentShape.noOverlay,
      );

      await tester.pumpWidget(_buildRangeApp(sliderTheme, values: const RangeValues(0.0, 1.0)));

      final MaterialInkController material = Material.of(tester.element(find.byType(RangeSlider)));

      // The track rectangle begins at 10 pixels from the left of the screen and ends 10 pixels from the right
      // (790 pixels from the left). The main check here it that the track itself should be centered on
      // the 800 pixel-wide screen.
      expect(
        material,
        paints
          // active track RRect. Starts 10 pixels from left of screen.
          ..rrect(
            rrect: RRect.fromLTRBAndCorners(
              10.0,
              298.0,
              10.0,
              302.0,
              topLeft: const Radius.circular(2.0),
              bottomLeft: const Radius.circular(2.0),
            ),
          )
          // inactive track RRect. Ends 10 pixels from right of screen.
          ..rrect(
            rrect: RRect.fromLTRBAndCorners(
              790.0,
              298.0,
              790.0,
              302.0,
              topRight: const Radius.circular(2.0),
              bottomRight: const Radius.circular(2.0),
            ),
          )
          // active track RRect Start 10 pixels from left screen.
          ..rrect(rrect: RRect.fromLTRBR(8.0, 297.0, 792.0, 303.0, const Radius.circular(2.0)))
          // The thumb Left.
          ..circle(x: 10.0, y: 300.0, radius: 10.0)
          // The thumb Right.
          ..circle(x: 790.0, y: 300.0, radius: 10.0),
      );
    },
  );

  // Only the thumb, overlay, and tick mark have special shortcuts to provide
  // no-op or empty shapes.
  //
  // The track can also be skipped by providing 0 height.
  //
  // The value indicator can be skipped by passing the appropriate
  // [ShowValueIndicator].
  testWidgets('The slider can skip all of its component painting', (WidgetTester tester) async {
    // Pump a slider with all shapes skipped.
    await tester.pumpWidget(
      _buildApp(
        ThemeData().sliderTheme.copyWith(
          trackHeight: 0,
          overlayShape: SliderComponentShape.noOverlay,
          thumbShape: SliderComponentShape.noThumb,
          tickMarkShape: SliderTickMarkShape.noTickMark,
          showValueIndicator: ShowValueIndicator.never,
        ),
        value: 0.5,
        divisions: 4,
      ),
    );

    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

    expect(material, paintsExactlyCountTimes(#drawRect, 0));
    expect(material, paintsExactlyCountTimes(#drawCircle, 0));
    expect(material, paintsExactlyCountTimes(#drawPath, 0));
  });

  testWidgets('The slider can skip all component painting except the track', (
    WidgetTester tester,
  ) async {
    // Pump a slider with just a track.
    await tester.pumpWidget(
      _buildApp(
        ThemeData().sliderTheme.copyWith(
          overlayShape: SliderComponentShape.noOverlay,
          thumbShape: SliderComponentShape.noThumb,
          tickMarkShape: SliderTickMarkShape.noTickMark,
          showValueIndicator: ShowValueIndicator.never,
        ),
        value: 0.5,
        divisions: 4,
      ),
    );

    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

    // Only 2 track segments.
    expect(material, paintsExactlyCountTimes(#drawRRect, 2));
    expect(material, paintsExactlyCountTimes(#drawCircle, 0));
    expect(material, paintsExactlyCountTimes(#drawPath, 0));
  });

  testWidgets('The slider can skip all component painting except the tick marks', (
    WidgetTester tester,
  ) async {
    // Pump a slider with just tick marks.
    await tester.pumpWidget(
      _buildApp(
        ThemeData().sliderTheme.copyWith(
          trackHeight: 0,
          overlayShape: SliderComponentShape.noOverlay,
          thumbShape: SliderComponentShape.noThumb,
          showValueIndicator: ShowValueIndicator.never,
          // When the track is hidden to 0 height, a tick mark radius
          // must be provided to get a non-zero radius.
          tickMarkShape: const RoundSliderTickMarkShape(tickMarkRadius: 1),
        ),
        value: 0.5,
        divisions: 4,
      ),
    );

    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

    // Only 5 tick marks.
    expect(material, paintsExactlyCountTimes(#drawRect, 0));
    expect(material, paintsExactlyCountTimes(#drawCircle, 5));
    expect(material, paintsExactlyCountTimes(#drawPath, 0));
  });

  testWidgets('The slider can skip all component painting except the thumb', (
    WidgetTester tester,
  ) async {
    debugDisableShadows = false;
    try {
      // Pump a slider with just a thumb.
      await tester.pumpWidget(
        _buildApp(
          ThemeData().sliderTheme.copyWith(
            trackHeight: 0,
            overlayShape: SliderComponentShape.noOverlay,
            tickMarkShape: SliderTickMarkShape.noTickMark,
            showValueIndicator: ShowValueIndicator.never,
          ),
          value: 0.5,
          divisions: 4,
        ),
      );

      final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

      // Only 1 thumb.
      expect(material, paintsExactlyCountTimes(#drawRect, 0));
      expect(material, paintsExactlyCountTimes(#drawCircle, 1));
      expect(material, paintsExactlyCountTimes(#drawPath, 0));
    } finally {
      debugDisableShadows = true;
    }
  });

  testWidgets('The slider can skip all component painting except the overlay', (
    WidgetTester tester,
  ) async {
    // Pump a slider with just an overlay.
    await tester.pumpWidget(
      _buildApp(
        ThemeData().sliderTheme.copyWith(
          trackHeight: 0,
          thumbShape: SliderComponentShape.noThumb,
          tickMarkShape: SliderTickMarkShape.noTickMark,
          showValueIndicator: ShowValueIndicator.never,
        ),
        value: 0.5,
        divisions: 4,
      ),
    );

    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

    // Tap the center of the track and wait for animations to finish.
    final Offset center = tester.getCenter(find.byType(Slider));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pumpAndSettle();

    // Only 1 overlay.
    expect(material, paintsExactlyCountTimes(#drawRect, 0));
    expect(material, paintsExactlyCountTimes(#drawCircle, 1));
    expect(material, paintsExactlyCountTimes(#drawPath, 0));

    await gesture.up();
  });

  testWidgets('The slider can skip all component painting except the value indicator', (
    WidgetTester tester,
  ) async {
    // Pump a slider with just a value indicator.
    await tester.pumpWidget(
      _buildApp(
        ThemeData().sliderTheme.copyWith(
          trackHeight: 0,
          overlayShape: SliderComponentShape.noOverlay,
          thumbShape: SliderComponentShape.noThumb,
          tickMarkShape: SliderTickMarkShape.noTickMark,
          showValueIndicator: ShowValueIndicator.always,
        ),
        value: 0.5,
        divisions: 4,
      ),
    );

    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));
    final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

    // Tap the center of the track and wait for animations to finish.
    final Offset center = tester.getCenter(find.byType(Slider));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pumpAndSettle();

    // Only 1 value indicator.
    expect(material, paintsExactlyCountTimes(#drawRect, 0));
    expect(material, paintsExactlyCountTimes(#drawCircle, 0));
    expect(valueIndicatorBox, paintsExactlyCountTimes(#drawPath, 1));

    await gesture.up();
  });

  testWidgets('PaddleSliderValueIndicatorShape skips all painting at zero scale', (
    WidgetTester tester,
  ) async {
    // Pump a slider with just a value indicator.
    await tester.pumpWidget(
      _buildApp(
        ThemeData().sliderTheme.copyWith(
          trackHeight: 0,
          overlayShape: SliderComponentShape.noOverlay,
          thumbShape: SliderComponentShape.noThumb,
          tickMarkShape: SliderTickMarkShape.noTickMark,
          showValueIndicator: ShowValueIndicator.always,
          valueIndicatorShape: const PaddleSliderValueIndicatorShape(),
        ),
        value: 0.5,
        divisions: 4,
      ),
    );

    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));
    final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

    // Tap the center of the track to kick off the animation of the value indicator.
    final Offset center = tester.getCenter(find.byType(Slider));
    final TestGesture gesture = await tester.startGesture(center);

    // Nothing to paint at scale 0.
    await tester.pump();
    expect(material, paintsExactlyCountTimes(#drawRect, 0));
    expect(material, paintsExactlyCountTimes(#drawCircle, 0));
    expect(valueIndicatorBox, paintsExactlyCountTimes(#drawPath, 0));

    // Painting a path for the value indicator.
    await tester.pump(const Duration(milliseconds: 16));
    expect(valueIndicatorBox, paintsExactlyCountTimes(#drawPath, 1));

    await gesture.up();
  });

  testWidgets('Default slider value indicator shape skips all painting at zero scale', (
    WidgetTester tester,
  ) async {
    // Pump a slider with just a value indicator.
    await tester.pumpWidget(
      _buildApp(
        ThemeData().sliderTheme.copyWith(
          trackHeight: 0,
          overlayShape: SliderComponentShape.noOverlay,
          thumbShape: SliderComponentShape.noThumb,
          tickMarkShape: SliderTickMarkShape.noTickMark,
          showValueIndicator: ShowValueIndicator.always,
        ),
        value: 0.5,
        divisions: 4,
      ),
    );

    final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

    // Tap the center of the track to kick off the animation of the value indicator.
    final Offset center = tester.getCenter(find.byType(Slider));
    final TestGesture gesture = await tester.startGesture(center);

    // Nothing to paint at scale 0.
    await tester.pump();
    expect(valueIndicatorBox, paintsExactlyCountTimes(#drawPath, 0));

    // Painting a path for the value indicator.
    await tester.pump(const Duration(milliseconds: 16));
    expect(valueIndicatorBox, paintsExactlyCountTimes(#drawPath, 1));

    await gesture.up();
  });

  testWidgets('Default paddle range slider value indicator shape draws correctly', (
    WidgetTester tester,
  ) async {
    debugDisableShadows = false;
    try {
      final ThemeData theme = ThemeData(
        platform: TargetPlatform.android,
        primarySwatch: Colors.blue,
      );
      final SliderThemeData sliderTheme = theme.sliderTheme.copyWith(
        thumbColor: Colors.red.shade500,
        showValueIndicator: ShowValueIndicator.always,
        rangeValueIndicatorShape: const PaddleRangeSliderValueIndicatorShape(),
      );

      await tester.pumpWidget(_buildRangeApp(sliderTheme));

      final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

      final Offset center = tester.getCenter(find.byType(RangeSlider));
      final TestGesture gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();
      expect(
        valueIndicatorBox,
        paints
          // physical model
          ..rrect()
          ..rrect(
            rrect: RRect.fromLTRBAndCorners(
              24.0,
              298.0,
              24.0,
              302.0,
              topLeft: const Radius.circular(2.0),
              bottomLeft: const Radius.circular(2.0),
            ),
          )
          ..rrect(
            rrect: RRect.fromLTRBAndCorners(
              24.0,
              298.0,
              776.0,
              302.0,
              topRight: const Radius.circular(2.0),
              bottomRight: const Radius.circular(2.0),
            ),
          )
          ..rrect(rrect: RRect.fromLTRBR(22.0, 297.0, 26.0, 303.0, const Radius.circular(2.0)))
          ..circle(x: 24.0, y: 300.0)
          ..shadow(elevation: 1.0)
          ..circle(x: 24.0, y: 300.0)
          ..shadow(elevation: 6.0)
          ..circle(x: 24.0, y: 300.0),
      );

      await gesture.up();
    } finally {
      debugDisableShadows = true;
    }
  });

  testWidgets(
    'Default paddle range slider value indicator shape draws correctly with debugDisableShadows',
    (WidgetTester tester) async {
      debugDisableShadows = true;
      final ThemeData theme = ThemeData(
        platform: TargetPlatform.android,
        primarySwatch: Colors.blue,
      );
      final SliderThemeData sliderTheme = theme.sliderTheme.copyWith(
        thumbColor: Colors.red.shade500,
        showValueIndicator: ShowValueIndicator.always,
        rangeValueIndicatorShape: const PaddleRangeSliderValueIndicatorShape(),
      );

      await tester.pumpWidget(_buildRangeApp(sliderTheme));

      final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

      final Offset center = tester.getCenter(find.byType(RangeSlider));
      final TestGesture gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();
      expect(
        valueIndicatorBox,
        paints
          // physical model
          ..rrect()
          ..rrect(
            rrect: RRect.fromLTRBAndCorners(
              24.0,
              298.0,
              24.0,
              302.0,
              topLeft: const Radius.circular(2.0),
              bottomLeft: const Radius.circular(2.0),
            ),
          )
          ..rrect(
            rrect: RRect.fromLTRBAndCorners(
              24.0,
              298.0,
              776.0,
              302.0,
              topRight: const Radius.circular(2.0),
              bottomRight: const Radius.circular(2.0),
            ),
          )
          ..rrect(rrect: RRect.fromLTRBR(22.0, 297.0, 26.0, 303.0, const Radius.circular(2)))
          ..circle(x: 24.0, y: 300.0)
          ..path(strokeWidth: 1.0 * 2.0, color: Colors.black)
          ..circle(x: 24.0, y: 300.0)
          ..path(strokeWidth: 6.0 * 2.0, color: Colors.black)
          ..circle(x: 24.0, y: 300.0),
      );

      await gesture.up();
    },
  );

  testWidgets('PaddleRangeSliderValueIndicatorShape skips all painting at zero scale', (
    WidgetTester tester,
  ) async {
    debugDisableShadows = false;
    try {
      // Pump a slider with just a value indicator.
      await tester.pumpWidget(
        _buildRangeApp(
          ThemeData().sliderTheme.copyWith(
            trackHeight: 0,
            rangeValueIndicatorShape: const PaddleRangeSliderValueIndicatorShape(),
          ),
          values: const RangeValues(0, 0.5),
          divisions: 4,
        ),
      );

      //  final RenderBox sliderBox = tester.firstRenderObject<RenderBox>(find.byType(RangeSlider));
      final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

      // Tap the center of the track to kick off the animation of the value indicator.
      final Offset center = tester.getCenter(find.byType(RangeSlider));
      final TestGesture gesture = await tester.startGesture(center);

      // No value indicator path to paint at scale 0.
      await tester.pump();
      expect(valueIndicatorBox, paintsExactlyCountTimes(#drawPath, 0));

      // Painting a path for each value indicator.
      await tester.pump(const Duration(milliseconds: 16));
      expect(valueIndicatorBox, paintsExactlyCountTimes(#drawPath, 2));

      await gesture.up();
    } finally {
      debugDisableShadows = true;
    }
  });

  testWidgets('Default range indicator shape skips all painting at zero scale', (
    WidgetTester tester,
  ) async {
    debugDisableShadows = false;
    try {
      // Pump a slider with just a value indicator.
      await tester.pumpWidget(
        _buildRangeApp(
          ThemeData().sliderTheme.copyWith(
            trackHeight: 0,
            overlayShape: SliderComponentShape.noOverlay,
            thumbShape: SliderComponentShape.noThumb,
            tickMarkShape: SliderTickMarkShape.noTickMark,
            showValueIndicator: ShowValueIndicator.always,
          ),
          values: const RangeValues(0, 0.5),
          divisions: 4,
        ),
      );

      final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

      // Tap the center of the track to kick off the animation of the value indicator.
      final Offset center = tester.getCenter(find.byType(RangeSlider));
      final TestGesture gesture = await tester.startGesture(center);

      // No value indicator path to paint at scale 0.
      await tester.pump();
      expect(valueIndicatorBox, paintsExactlyCountTimes(#drawPath, 0));

      // Painting a path for each value indicator.
      await tester.pump(const Duration(milliseconds: 16));
      expect(valueIndicatorBox, paintsExactlyCountTimes(#drawPath, 2));

      await gesture.up();
    } finally {
      debugDisableShadows = true;
    }
  });

  testWidgets(
    'activeTrackRadius is taken into account when painting the border of the active track',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        _buildApp(
          value: 0.5,
          ThemeData().sliderTheme.copyWith(
            trackShape: const RoundedRectSliderTrackShapeWithCustomAdditionalActiveTrackHeight(
              additionalActiveTrackHeight: 10.0,
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();
      final Offset center = tester.getCenter(find.byType(Slider));
      final TestGesture gesture = await tester.startGesture(center);
      expect(
        find.byType(Slider),
        paints
          // Inactive track.
          ..rrect(rrect: RRect.fromLTRBR(398.0, 298.0, 776.0, 302.0, const Radius.circular(2.0)))
          // Active track.
          ..rrect(rrect: RRect.fromLTRBR(24.0, 293.0, 402.0, 307.0, const Radius.circular(7.0))),
      );

      // Finish gesture to release resources.
      await gesture.up();
      await tester.pumpAndSettle();
    },
  );

  testWidgets('The mouse cursor is themeable', (WidgetTester tester) async {
    await tester.pumpWidget(
      _buildApp(
        ThemeData().sliderTheme.copyWith(
          mouseCursor: const MaterialStatePropertyAll<MouseCursor>(SystemMouseCursors.text),
        ),
      ),
    );

    await tester.pumpAndSettle();
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byType(Slider)));
    await tester.pumpAndSettle();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );
  });

  testWidgets('SliderTheme.allowedInteraction is themeable', (WidgetTester tester) async {
    double value = 0.0;

    Widget buildApp({
      bool isAllowedInteractionInThemeNull = false,
      bool isAllowedInteractionInSliderNull = false,
    }) {
      return MaterialApp(
        home: Scaffold(
          body: Center(
            child: SliderTheme(
              data: ThemeData().sliderTheme.copyWith(
                allowedInteraction:
                    isAllowedInteractionInThemeNull ? null : SliderInteraction.slideOnly,
              ),
              child: StatefulBuilder(
                builder: (_, void Function(void Function()) setState) {
                  return Slider(
                    value: value,
                    allowedInteraction:
                        isAllowedInteractionInSliderNull ? null : SliderInteraction.tapOnly,
                    onChanged: (double newValue) {
                      setState(() {
                        value = newValue;
                      });
                    },
                  );
                },
              ),
            ),
          ),
        ),
      );
    }

    final TestGesture gesture = await tester.createGesture();

    // when theme and parameter are specified, parameter is used [tapOnly].
    await tester.pumpWidget(buildApp());
    // tap is allowed.
    value = 0.0;
    await gesture.down(tester.getCenter(find.byType(Slider)));
    await tester.pump();
    expect(value, equals(0.5)); // changes
    await gesture.up();
    // slide isn't allowed.
    value = 0.0;
    await gesture.down(tester.getCenter(find.byType(Slider)));
    await tester.pump();
    await gesture.moveBy(const Offset(50, 0));
    expect(value, equals(0.0)); // no change
    await gesture.up();

    // when only parameter is specified, parameter is used [tapOnly].
    await tester.pumpWidget(buildApp(isAllowedInteractionInThemeNull: true));
    // tap is allowed.
    value = 0.0;
    await gesture.down(tester.getCenter(find.byType(Slider)));
    await tester.pump();
    expect(value, equals(0.5)); // changes
    await gesture.up();
    // slide isn't allowed.
    value = 0.0;
    await gesture.down(tester.getCenter(find.byType(Slider)));
    await tester.pump();
    await gesture.moveBy(const Offset(50, 0));
    expect(value, equals(0.0)); // no change
    await gesture.up();

    // when theme is specified but parameter is null, theme is used [slideOnly].
    await tester.pumpWidget(buildApp(isAllowedInteractionInSliderNull: true));
    // tap isn't allowed.
    value = 0.0;
    await gesture.down(tester.getCenter(find.byType(Slider)));
    await tester.pump();
    expect(value, equals(0.0)); // no change
    await gesture.up();
    // slide isn't allowed.
    value = 0.0;
    await gesture.down(tester.getCenter(find.byType(Slider)));
    await tester.pump();
    await gesture.moveBy(const Offset(50, 0));
    expect(value, greaterThan(0.0)); // changes
    await gesture.up();

    // when both theme and parameter are null, default is used [tapAndSlide].
    await tester.pumpWidget(
      buildApp(isAllowedInteractionInSliderNull: true, isAllowedInteractionInThemeNull: true),
    );
    // tap is allowed.
    value = 0.0;
    await gesture.down(tester.getCenter(find.byType(Slider)));
    await tester.pump();
    expect(value, equals(0.5));
    await gesture.up();
    // slide is allowed.
    value = 0.0;
    await gesture.down(tester.getCenter(find.byType(Slider)));
    await tester.pump();
    await gesture.moveBy(const Offset(50, 0));
    expect(value, greaterThan(0.0)); // changes
    await gesture.up();
  });

  testWidgets('Default value indicator color', (WidgetTester tester) async {
    debugDisableShadows = false;
    try {
      final ThemeData theme = ThemeData(useMaterial3: true, platform: TargetPlatform.android);
      Widget buildApp(
        String value, {
        double sliderValue = 0.5,
        TextScaler textScaler = TextScaler.noScaling,
      }) {
        return MaterialApp(
          theme: theme,
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: MediaQuery(
              data: MediaQueryData(textScaler: textScaler),
              child: Material(
                child: Row(
                  children: <Widget>[
                    Expanded(
                      child: Slider(
                        value: sliderValue,
                        label: value,
                        divisions: 3,
                        onChanged: (double d) {},
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp('1'));

      final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

      final Offset center = tester.getCenter(find.byType(Slider));
      final TestGesture gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();
      expect(
        valueIndicatorBox,
        paints
          ..rrect(color: const Color(0xfffef7ff))
          ..rrect(color: const Color(0xffe6e0e9))
          ..rrect(color: const Color(0xff6750a4))
          ..path(color: Color(theme.colorScheme.primary.value)),
      );

      // Finish gesture to release resources.
      await gesture.up();
      await tester.pumpAndSettle();
    } finally {
      debugDisableShadows = true;
    }
  });

  testWidgets(
    'RectangularSliderValueIndicatorShape supports SliderTheme.valueIndicatorStrokeColor',
    (WidgetTester tester) async {
      final ThemeData theme = ThemeData(
        sliderTheme: const SliderThemeData(
          showValueIndicator: ShowValueIndicator.always,
          valueIndicatorShape: RectangularSliderValueIndicatorShape(),
          valueIndicatorColor: Color(0xff000001),
          valueIndicatorStrokeColor: Color(0xff000002),
        ),
      );

      const double value = 0.5;

      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: Material(
            child: Center(
              child: Slider(value: value, label: '$value', onChanged: (double newValue) {}),
            ),
          ),
        ),
      );

      final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

      final Offset center = tester.getCenter(find.byType(Slider));
      await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();

      expect(
        valueIndicatorBox,
        paints
          ..path(color: theme.colorScheme.shadow) // shadow
          ..path(color: theme.sliderTheme.valueIndicatorStrokeColor)
          ..path(color: theme.sliderTheme.valueIndicatorColor),
      );
    },
  );

  testWidgets('PaddleSliderValueIndicatorShape supports SliderTheme.valueIndicatorStrokeColor', (
    WidgetTester tester,
  ) async {
    final ThemeData theme = ThemeData(
      sliderTheme: const SliderThemeData(
        showValueIndicator: ShowValueIndicator.always,
        valueIndicatorShape: PaddleSliderValueIndicatorShape(),
        valueIndicatorColor: Color(0xff000001),
        valueIndicatorStrokeColor: Color(0xff000002),
      ),
    );

    const double value = 0.5;

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: Slider(value: value, label: '$value', onChanged: (double newValue) {}),
          ),
        ),
      ),
    );

    final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

    final Offset center = tester.getCenter(find.byType(Slider));
    await tester.startGesture(center);
    // Wait for value indicator animation to finish.
    await tester.pumpAndSettle();

    expect(
      valueIndicatorBox,
      paints
        ..path(color: theme.colorScheme.shadow) // shadow
        ..path(color: theme.sliderTheme.valueIndicatorStrokeColor)
        ..path(color: theme.sliderTheme.valueIndicatorColor),
    );
  });

  testWidgets('DropSliderValueIndicatorShape supports SliderTheme.valueIndicatorStrokeColor', (
    WidgetTester tester,
  ) async {
    final ThemeData theme = ThemeData(
      sliderTheme: const SliderThemeData(
        showValueIndicator: ShowValueIndicator.always,
        valueIndicatorShape: DropSliderValueIndicatorShape(),
        valueIndicatorColor: Color(0xff000001),
        valueIndicatorStrokeColor: Color(0xff000002),
      ),
    );

    const double value = 0.5;

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: Slider(value: value, label: '$value', onChanged: (double newValue) {}),
          ),
        ),
      ),
    );

    final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

    final Offset center = tester.getCenter(find.byType(Slider));
    await tester.startGesture(center);
    // Wait for value indicator animation to finish.
    await tester.pumpAndSettle();

    expect(
      valueIndicatorBox,
      paints
        ..path(color: theme.colorScheme.shadow) // shadow
        ..path(color: theme.sliderTheme.valueIndicatorStrokeColor)
        ..path(color: theme.sliderTheme.valueIndicatorColor),
    );
  });

  testWidgets(
    'RectangularRangeSliderValueIndicatorShape supports SliderTheme.valueIndicatorStrokeColor',
    (WidgetTester tester) async {
      final ThemeData theme = ThemeData(
        sliderTheme: const SliderThemeData(
          showValueIndicator: ShowValueIndicator.always,
          rangeValueIndicatorShape: RectangularRangeSliderValueIndicatorShape(),
          valueIndicatorColor: Color(0xff000001),
          valueIndicatorStrokeColor: Color(0xff000002),
        ),
      );

      RangeValues values = const RangeValues(0, 0.5);

      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: Material(
            child: Center(
              child: RangeSlider(
                values: values,
                labels: RangeLabels(values.start.toString(), values.end.toString()),
                onChanged: (RangeValues val) {
                  values = val;
                },
              ),
            ),
          ),
        ),
      );

      final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

      final Offset center = tester.getCenter(find.byType(RangeSlider));
      final TestGesture gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();

      expect(
        valueIndicatorBox,
        paints
          ..path(color: theme.colorScheme.shadow) // shadow
          ..path(color: theme.colorScheme.shadow) // shadow
          ..path(color: theme.sliderTheme.valueIndicatorStrokeColor)
          ..path(color: theme.sliderTheme.valueIndicatorColor)
          ..path(color: theme.sliderTheme.valueIndicatorStrokeColor)
          ..path(color: theme.sliderTheme.valueIndicatorColor),
      );

      await gesture.up();
    },
  );

  testWidgets(
    'RectangularRangeSliderValueIndicatorShape supports SliderTheme.valueIndicatorStrokeColor on overlapping indicator',
    (WidgetTester tester) async {
      final ThemeData theme = ThemeData(
        sliderTheme: const SliderThemeData(
          showValueIndicator: ShowValueIndicator.always,
          rangeValueIndicatorShape: RectangularRangeSliderValueIndicatorShape(),
          valueIndicatorColor: Color(0xff000001),
          valueIndicatorStrokeColor: Color(0xff000002),
          overlappingShapeStrokeColor: Color(0xff000003),
        ),
      );

      RangeValues values = const RangeValues(0.0, 0.0);

      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: Material(
            child: Center(
              child: RangeSlider(
                values: values,
                labels: RangeLabels(values.start.toString(), values.end.toString()),
                onChanged: (RangeValues val) {
                  values = val;
                },
              ),
            ),
          ),
        ),
      );

      final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

      final Offset center = tester.getCenter(find.byType(RangeSlider));
      final TestGesture gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();

      expect(
        valueIndicatorBox,
        paints
          ..path(color: theme.colorScheme.shadow) // shadow
          ..path(color: theme.colorScheme.shadow) // shadow
          ..path(color: theme.sliderTheme.valueIndicatorStrokeColor)
          ..path(color: theme.sliderTheme.valueIndicatorColor)
          ..path(color: theme.sliderTheme.overlappingShapeStrokeColor)
          ..path(color: theme.sliderTheme.valueIndicatorColor),
      );

      await gesture.up();
    },
  );

  testWidgets(
    'PaddleRangeSliderValueIndicatorShape supports SliderTheme.valueIndicatorStrokeColor',
    (WidgetTester tester) async {
      final ThemeData theme = ThemeData(
        sliderTheme: const SliderThemeData(
          showValueIndicator: ShowValueIndicator.always,
          rangeValueIndicatorShape: PaddleRangeSliderValueIndicatorShape(),
          valueIndicatorColor: Color(0xff000001),
          valueIndicatorStrokeColor: Color(0xff000002),
        ),
      );

      RangeValues values = const RangeValues(0, 0.5);

      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: Material(
            child: Center(
              child: RangeSlider(
                values: values,
                labels: RangeLabels(values.start.toString(), values.end.toString()),
                onChanged: (RangeValues val) {
                  values = val;
                },
              ),
            ),
          ),
        ),
      );

      final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

      final Offset center = tester.getCenter(find.byType(RangeSlider));
      final TestGesture gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();

      expect(
        valueIndicatorBox,
        paints
          ..path(color: theme.colorScheme.shadow) // shadow
          ..path(color: theme.colorScheme.shadow) // shadow
          ..path(color: theme.sliderTheme.valueIndicatorStrokeColor)
          ..path(color: theme.sliderTheme.valueIndicatorColor)
          ..path(color: theme.sliderTheme.valueIndicatorStrokeColor)
          ..path(color: theme.sliderTheme.valueIndicatorColor),
      );

      await gesture.up();
    },
  );

  testWidgets(
    'PaddleRangeSliderValueIndicatorShape supports SliderTheme.valueIndicatorStrokeColor on overlapping indicator',
    (WidgetTester tester) async {
      final ThemeData theme = ThemeData(
        sliderTheme: const SliderThemeData(
          showValueIndicator: ShowValueIndicator.always,
          rangeValueIndicatorShape: PaddleRangeSliderValueIndicatorShape(),
          valueIndicatorColor: Color(0xff000001),
          valueIndicatorStrokeColor: Color(0xff000002),
          overlappingShapeStrokeColor: Color(0xff000003),
        ),
      );

      RangeValues values = const RangeValues(0, 0);

      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: Material(
            child: Center(
              child: RangeSlider(
                values: values,
                labels: RangeLabels(values.start.toString(), values.end.toString()),
                onChanged: (RangeValues val) {
                  values = val;
                },
              ),
            ),
          ),
        ),
      );

      final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

      final Offset center = tester.getCenter(find.byType(RangeSlider));
      final TestGesture gesture = await tester.startGesture(center);
      // Wait for value indicator animation to finish.
      await tester.pumpAndSettle();

      expect(
        valueIndicatorBox,
        paints
          ..path(color: theme.colorScheme.shadow) // shadow
          ..path(color: theme.colorScheme.shadow) // shadow
          ..path(color: theme.sliderTheme.valueIndicatorStrokeColor)
          ..path(color: theme.sliderTheme.valueIndicatorColor)
          ..path(color: theme.sliderTheme.overlappingShapeStrokeColor)
          ..path(color: theme.sliderTheme.valueIndicatorColor),
      );

      await gesture.up();
    },
  );

  group('RoundedRectSliderTrackShape', () {
    testWidgets(
      'Only draw active track if thumb center is higher than trackRect.left and track radius',
      (WidgetTester tester) async {
        const SliderThemeData sliderTheme = SliderThemeData(
          trackShape: RoundedRectSliderTrackShape(),
        );
        await tester.pumpWidget(_buildApp(sliderTheme));

        MaterialInkController material = Material.of(tester.element(find.byType(Slider)));
        expect(
          material,
          paints
            // Inactive track.
            ..rrect(rrect: RRect.fromLTRBR(22.0, 298.0, 776.0, 302.0, const Radius.circular(2.0))),
        );

        await tester.pumpWidget(_buildApp(sliderTheme, value: 0.025));

        material = Material.of(tester.element(find.byType(Slider)));
        expect(
          material,
          paints
            // Inactive track.
            ..rrect(rrect: RRect.fromLTRBR(40.8, 298.0, 776.0, 302.0, const Radius.circular(2.0)))
            // Active track.
            ..rrect(rrect: RRect.fromLTRBR(24.0, 297.0, 44.8, 303.0, const Radius.circular(3.0))),
        );
      },
    );

    testWidgets(
      'Only draw inactive track if thumb center is lower than trackRect.right and track radius',
      (WidgetTester tester) async {
        const SliderThemeData sliderTheme = SliderThemeData(
          trackShape: RoundedRectSliderTrackShape(),
        );
        await tester.pumpWidget(_buildApp(sliderTheme, value: 1.0));

        MaterialInkController material = Material.of(tester.element(find.byType(Slider)));
        expect(
          material,
          paints
            // Active track.
            ..rrect(rrect: RRect.fromLTRBR(24.0, 297.0, 778.0, 303.0, const Radius.circular(3.0))),
        );

        await tester.pumpWidget(_buildApp(sliderTheme, value: 0.975));

        material = Material.of(tester.element(find.byType(Slider)));
        expect(
          material,
          paints
            // Inactive track.
            ..rrect(rrect: RRect.fromLTRBR(755.2, 298.0, 776.0, 302.0, const Radius.circular(2.0)))
            // Active track.
            ..rrect(rrect: RRect.fromLTRBR(24.0, 297.0, 759.2, 303.0, const Radius.circular(3.0))),
        );
      },
    );
  });

  testWidgets('Track shape isRounded defaults', (WidgetTester tester) async {
    expect(const RectangularSliderTrackShape().isRounded, isFalse);
    expect(const RoundedRectSliderTrackShape().isRounded, isTrue);
    expect(const RectangularRangeSliderTrackShape().isRounded, isFalse);
    expect(const RoundedRectRangeSliderTrackShape().isRounded, isTrue);
  });

  testWidgets('SliderThemeData.padding can override the default Slider padding', (
    WidgetTester tester,
  ) async {
    Widget buildSlider({EdgeInsetsGeometry? padding}) {
      return MaterialApp(
        theme: ThemeData(sliderTheme: SliderThemeData(padding: padding)),
        home: Material(
          child: Center(
            child: IntrinsicHeight(child: Slider(value: 0.5, onChanged: (double value) {})),
          ),
        ),
      );
    }

    RenderBox sliderRenderBox() {
      return tester.allRenderObjects.firstWhere(
            (RenderObject object) => object.runtimeType.toString() == '_RenderSlider',
          )
          as RenderBox;
    }

    // Test Slider height and tracks spacing with zero padding.
    await tester.pumpWidget(buildSlider(padding: EdgeInsets.zero));
    await tester.pumpAndSettle();

    // The height equals to the default thumb height.
    expect(sliderRenderBox().size, const Size(800, 20));
    expect(
      find.byType(Slider),
      paints
        // Inactive track.
        ..rrect(rrect: RRect.fromLTRBR(398.0, 8.0, 800.0, 12.0, const Radius.circular(2.0)))
        // Active track.
        ..rrect(rrect: RRect.fromLTRBR(0.0, 7.0, 402.0, 13.0, const Radius.circular(3.0))),
    );

    // Test Slider height and tracks spacing with directional padding.
    const double startPadding = 100;
    const double endPadding = 20;
    await tester.pumpWidget(
      buildSlider(padding: const EdgeInsetsDirectional.only(start: startPadding, end: endPadding)),
    );
    await tester.pumpAndSettle();

    expect(sliderRenderBox().size, const Size(800 - startPadding - endPadding, 20));
    expect(
      find.byType(Slider),
      paints
        // Inactive track.
        ..rrect(rrect: RRect.fromLTRBR(338.0, 8.0, 680.0, 12.0, const Radius.circular(2.0)))
        // Active track.
        ..rrect(rrect: RRect.fromLTRBR(0.0, 7.0, 342.0, 13.0, const Radius.circular(3.0))),
    );

    // Test Slider height and tracks spacing with top and bottom padding.
    const double topPadding = 100;
    const double bottomPadding = 20;
    const double trackHeight = 20;
    await tester.pumpWidget(
      buildSlider(
        padding: const EdgeInsetsDirectional.only(top: topPadding, bottom: bottomPadding),
      ),
    );
    await tester.pumpAndSettle();

    expect(
      tester.getSize(find.byType(Slider)),
      const Size(800, topPadding + trackHeight + bottomPadding),
    );
    expect(sliderRenderBox().size, const Size(800, 20));
    expect(
      find.byType(Slider),
      paints
        // Inactive track.
        ..rrect(rrect: RRect.fromLTRBR(398.0, 8.0, 800.0, 12.0, const Radius.circular(2.0)))
        // Active track.
        ..rrect(rrect: RRect.fromLTRBR(0.0, 7.0, 402.0, 13.0, const Radius.circular(3.0))),
    );
  });

  testWidgets('Can customize track gap when year2023 is false', (WidgetTester tester) async {
    Widget buildSlider({double? trackGap}) {
      return MaterialApp(
        theme: ThemeData(sliderTheme: SliderThemeData(trackGap: trackGap)),
        home: Material(
          child: Center(child: Slider(year2023: false, value: 0.5, onChanged: (double value) {})),
        ),
      );
    }

    await tester.pumpWidget(buildSlider(trackGap: 0));

    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

    // Test default track shape.
    const Radius trackOuterCornerRadius = Radius.circular(8.0);
    const Radius trackInnerCornerRadius = Radius.circular(2.0);
    expect(
      material,
      paints
        // Active track.
        ..rrect(
          rrect: RRect.fromLTRBAndCorners(
            24.0,
            292.0,
            400.0,
            308.0,
            topLeft: trackOuterCornerRadius,
            topRight: trackInnerCornerRadius,
            bottomRight: trackInnerCornerRadius,
            bottomLeft: trackOuterCornerRadius,
          ),
        )
        // Inactive track.
        ..rrect(
          rrect: RRect.fromLTRBAndCorners(
            400.0,
            292.0,
            776.0,
            308.0,
            topLeft: trackInnerCornerRadius,
            topRight: trackOuterCornerRadius,
            bottomRight: trackOuterCornerRadius,
            bottomLeft: trackInnerCornerRadius,
          ),
        ),
    );

    await tester.pumpWidget(buildSlider(trackGap: 10));
    await tester.pumpAndSettle();
    expect(
      material,
      paints
        // Active track.
        ..rrect(
          rrect: RRect.fromLTRBAndCorners(
            24.0,
            292.0,
            390.0,
            308.0,
            topLeft: trackOuterCornerRadius,
            topRight: trackInnerCornerRadius,
            bottomRight: trackInnerCornerRadius,
            bottomLeft: trackOuterCornerRadius,
          ),
        )
        // Inactive track.
        ..rrect(
          rrect: RRect.fromLTRBAndCorners(
            410.0,
            292.0,
            776.0,
            308.0,
            topLeft: trackInnerCornerRadius,
            topRight: trackOuterCornerRadius,
            bottomRight: trackOuterCornerRadius,
            bottomLeft: trackInnerCornerRadius,
          ),
        ),
    );
  });

  testWidgets('Can customize thumb size when year2023 is false', (WidgetTester tester) async {
    Widget buildSlider({WidgetStateProperty<Size?>? thumbSize}) {
      return MaterialApp(
        theme: ThemeData(sliderTheme: SliderThemeData(thumbSize: thumbSize)),
        home: Material(
          child: Center(child: Slider(year2023: false, value: 0.5, onChanged: (double value) {})),
        ),
      );
    }

    await tester.pumpWidget(
      buildSlider(thumbSize: const WidgetStatePropertyAll<Size>(Size(20, 20))),
    );

    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));
    expect(
      material,
      paints
        ..circle()
        ..rrect(rrect: RRect.fromLTRBR(390.0, 290.0, 410.0, 310.0, const Radius.circular(10.0))),
    );

    await tester.pumpWidget(
      buildSlider(
        thumbSize: const WidgetStateProperty<Size?>.fromMap(<WidgetStatesConstraint, Size>{
          WidgetState.pressed: Size(20, 20),
          WidgetState.any: Size(10, 10),
        }),
      ),
    );
    await tester.pumpAndSettle();

    expect(
      material,
      paints
        ..circle()
        ..rrect(rrect: RRect.fromLTRBR(395.0, 295.0, 405.0, 305.0, const Radius.circular(5.0))),
    );

    final Offset center = tester.getCenter(find.byType(Slider));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pumpAndSettle();

    expect(
      material,
      paints
        ..circle()
        ..rrect(rrect: RRect.fromLTRBR(390.0, 295.0, 410.0, 305.0, const Radius.circular(5.0))),
    );

    await gesture.up();
    await tester.pumpAndSettle();
  });

  testWidgets('Opt into 2024 Slider appearance with SliderThemeData.year2023', (
    WidgetTester tester,
  ) async {
    final ThemeData theme = ThemeData(sliderTheme: const SliderThemeData(year2023: false));
    final ColorScheme colorScheme = theme.colorScheme;
    final Color activeTrackColor = colorScheme.primary;
    final Color inactiveTrackColor = colorScheme.secondaryContainer;
    const double value = 0.45;
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(child: Center(child: Slider(value: value, onChanged: (double value) {}))),
      ),
    );

    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

    // Test default track shape.
    const Radius trackOuterCornerRadius = Radius.circular(8.0);
    const Radius trackInnerCornderRadius = Radius.circular(2.0);
    expect(
      material,
      paints
        // Active track.
        ..rrect(
          rrect: RRect.fromLTRBAndCorners(
            24.0,
            292.0,
            356.4,
            308.0,
            topLeft: trackOuterCornerRadius,
            topRight: trackInnerCornderRadius,
            bottomRight: trackInnerCornderRadius,
            bottomLeft: trackOuterCornerRadius,
          ),
          color: activeTrackColor,
        )
        // Inctive track.
        ..rrect(
          rrect: RRect.fromLTRBAndCorners(
            368.4,
            292.0,
            776.0,
            308.0,
            topLeft: trackInnerCornderRadius,
            topRight: trackOuterCornerRadius,
            bottomRight: trackOuterCornerRadius,
            bottomLeft: trackInnerCornderRadius,
          ),
          color: inactiveTrackColor,
        ),
    );
  });

  testWidgets('Slider.year2023 overrides SliderThemeData.year2023', (WidgetTester tester) async {
    final ThemeData theme = ThemeData(sliderTheme: const SliderThemeData(year2023: false));
    final ColorScheme colorScheme = theme.colorScheme;
    final Color activeTrackColor = colorScheme.primary;
    final Color inactiveTrackColor = colorScheme.surfaceContainerHighest;
    const double value = 0.45;
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: Theme(
              data: theme,
              child: Slider(year2023: true, value: value, onChanged: (double value) {}),
            ),
          ),
        ),
      ),
    );

    final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));

    // Test default track shape.
    const Radius activeTrackCornerRadius = Radius.circular(3.0);
    const Radius inactiveTrackCornerRadius = Radius.circular(2.0);
    expect(
      material,
      paints
        // Active track.
        ..rrect(
          rrect: RRect.fromLTRBR(360.4, 298.0, 776.0, 302.0, inactiveTrackCornerRadius),
          color: inactiveTrackColor,
        )
        // Inctive track.
        ..rrect(
          rrect: RRect.fromLTRBR(24.0, 297.0, 364.4, 303.0, activeTrackCornerRadius),
          color: activeTrackColor,
        ),
    );
  });

  group('Material 2', () {
    // These tests are only relevant for Material 2. Once Material 2
    // support is deprecated and the APIs are removed, these tests
    // can be deleted.

    testWidgets('Slider defaults', (WidgetTester tester) async {
      debugDisableShadows = false;
      final ThemeData theme = ThemeData(useMaterial3: false);
      const double trackHeight = 4.0;
      final ColorScheme colorScheme = theme.colorScheme;
      final Color activeTrackColor = Color(colorScheme.primary.value);
      final Color inactiveTrackColor = colorScheme.primary.withOpacity(0.24);
      final Color secondaryActiveTrackColor = colorScheme.primary.withOpacity(0.54);
      final Color disabledActiveTrackColor = colorScheme.onSurface.withOpacity(0.32);
      final Color disabledInactiveTrackColor = colorScheme.onSurface.withOpacity(0.12);
      final Color disabledSecondaryActiveTrackColor = colorScheme.onSurface.withOpacity(0.12);
      final Color shadowColor = colorScheme.shadow;
      final Color thumbColor = Color(colorScheme.primary.value);
      final Color disabledThumbColor = Color.alphaBlend(
        colorScheme.onSurface.withOpacity(.38),
        colorScheme.surface,
      );
      final Color activeTickMarkColor = colorScheme.onPrimary.withOpacity(0.54);
      final Color inactiveTickMarkColor = colorScheme.primary.withOpacity(0.54);
      final Color disabledActiveTickMarkColor = colorScheme.onPrimary.withOpacity(0.12);
      final Color disabledInactiveTickMarkColor = colorScheme.onSurface.withOpacity(0.12);
      final Color valueIndicatorColor = Color.alphaBlend(
        colorScheme.onSurface.withOpacity(0.60),
        colorScheme.surface.withOpacity(0.90),
      );

      try {
        double value = 0.45;
        Widget buildApp({int? divisions, bool enabled = true}) {
          final ValueChanged<double>? onChanged =
              !enabled
                  ? null
                  : (double d) {
                    value = d;
                  };
          return MaterialApp(
            theme: theme,
            home: Material(
              child: Center(
                child: Slider(
                  value: value,
                  secondaryTrackValue: 0.75,
                  label: '$value',
                  divisions: divisions,
                  onChanged: onChanged,
                ),
              ),
            ),
          );
        }

        await tester.pumpWidget(buildApp());

        final MaterialInkController material = Material.of(tester.element(find.byType(Slider)));
        final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

        // Test default track height.
        const Radius radius = Radius.circular(trackHeight / 2);
        const Radius activatedRadius = Radius.circular((trackHeight + 2) / 2);
        expect(
          material,
          paints
            ..rrect(
              rrect: RRect.fromLTRBR(360.4, 298.0, 776.0, 302.0, radius),
              color: inactiveTrackColor,
            )
            ..rrect(
              rrect: RRect.fromLTRBR(24.0, 297.0, 364.4, 303.0, activatedRadius),
              color: activeTrackColor,
            ),
        );

        // Test default colors for enabled slider.
        expect(
          material,
          paints
            ..rrect(color: inactiveTrackColor)
            ..rrect(color: activeTrackColor)
            ..rrect(color: secondaryActiveTrackColor),
        );
        expect(material, paints..shadow(color: shadowColor));
        expect(material, paints..circle(color: thumbColor));
        expect(material, isNot(paints..circle(color: disabledThumbColor)));
        expect(material, isNot(paints..rrect(color: disabledActiveTrackColor)));
        expect(material, isNot(paints..rrect(color: disabledInactiveTrackColor)));
        expect(material, isNot(paints..rrect(color: disabledSecondaryActiveTrackColor)));
        expect(material, isNot(paints..circle(color: activeTickMarkColor)));
        expect(material, isNot(paints..circle(color: inactiveTickMarkColor)));

        // Test defaults colors for discrete slider.
        await tester.pumpWidget(buildApp(divisions: 3));
        expect(
          material,
          paints
            ..rrect(color: inactiveTrackColor)
            ..rrect(color: activeTrackColor)
            ..rrect(color: secondaryActiveTrackColor),
        );
        expect(
          material,
          paints
            ..circle(color: activeTickMarkColor)
            ..circle(color: activeTickMarkColor)
            ..circle(color: inactiveTickMarkColor)
            ..circle(color: inactiveTickMarkColor)
            ..shadow(color: Colors.black)
            ..circle(color: thumbColor),
        );
        expect(material, isNot(paints..circle(color: disabledThumbColor)));
        expect(material, isNot(paints..rrect(color: disabledActiveTrackColor)));
        expect(material, isNot(paints..rrect(color: disabledInactiveTrackColor)));
        expect(material, isNot(paints..rrect(color: disabledSecondaryActiveTrackColor)));

        // Test defaults colors for disabled slider.
        await tester.pumpWidget(buildApp(enabled: false));
        await tester.pumpAndSettle();
        expect(
          material,
          paints
            ..rrect(color: disabledInactiveTrackColor)
            ..rrect(color: disabledActiveTrackColor)
            ..rrect(color: disabledSecondaryActiveTrackColor),
        );
        expect(
          material,
          paints
            ..shadow(color: Colors.black)
            ..circle(color: disabledThumbColor),
        );
        expect(material, isNot(paints..circle(color: thumbColor)));
        expect(material, isNot(paints..rrect(color: activeTrackColor)));
        expect(material, isNot(paints..rrect(color: inactiveTrackColor)));
        expect(material, isNot(paints..rrect(color: secondaryActiveTrackColor)));

        // Test defaults colors for disabled discrete slider.
        await tester.pumpWidget(buildApp(divisions: 3, enabled: false));
        expect(
          material,
          paints
            ..circle(color: disabledActiveTickMarkColor)
            ..circle(color: disabledActiveTickMarkColor)
            ..circle(color: disabledInactiveTickMarkColor)
            ..circle(color: disabledInactiveTickMarkColor)
            ..shadow(color: Colors.black)
            ..circle(color: disabledThumbColor),
        );
        expect(material, isNot(paints..circle(color: thumbColor)));
        expect(material, isNot(paints..rrect(color: activeTrackColor)));
        expect(material, isNot(paints..rrect(color: inactiveTrackColor)));
        expect(material, isNot(paints..rrect(color: secondaryActiveTrackColor)));
        expect(material, isNot(paints..circle(color: activeTickMarkColor)));
        expect(material, isNot(paints..circle(color: inactiveTickMarkColor)));

        // Test the default color for value indicator.
        await tester.pumpWidget(buildApp(divisions: 3));
        final Offset center = tester.getCenter(find.byType(Slider));
        final TestGesture gesture = await tester.startGesture(center);
        // Wait for value indicator animation to finish.
        await tester.pumpAndSettle();
        expect(value, equals(2.0 / 3.0));
        expect(
          valueIndicatorBox,
          paints
            ..path(color: valueIndicatorColor)
            ..paragraph(),
        );
        await gesture.up();
        // Wait for value indicator animation to finish.
        await tester.pumpAndSettle();
      } finally {
        debugDisableShadows = true;
      }
    });

    testWidgets('Default value indicator color', (WidgetTester tester) async {
      debugDisableShadows = false;
      try {
        final ThemeData theme = ThemeData(useMaterial3: false, platform: TargetPlatform.android);
        Widget buildApp(
          String value, {
          double sliderValue = 0.5,
          TextScaler textScaler = TextScaler.noScaling,
        }) {
          return MaterialApp(
            theme: theme,
            home: MediaQuery(
              data: MediaQueryData(textScaler: textScaler),
              child: Material(
                child: Row(
                  children: <Widget>[
                    Expanded(
                      child: Slider(
                        value: sliderValue,
                        label: value,
                        divisions: 3,
                        onChanged: (double d) {},
                      ),
                    ),
                  ],
                ),
              ),
            ),
          );
        }

        await tester.pumpWidget(buildApp('1'));

        final RenderBox valueIndicatorBox = tester.renderObject(find.byType(Overlay));

        final Offset center = tester.getCenter(find.byType(Slider));
        final TestGesture gesture = await tester.startGesture(center);
        // Wait for value indicator animation to finish.
        await tester.pumpAndSettle();
        expect(
          valueIndicatorBox,
          paints
            ..rrect(color: const Color(0xfffafafa))
            ..rrect(color: const Color(0x3d2196f3))
            ..rrect(color: const Color(0xff2196f3))
            // Test that the value indicator text is painted with the correct color.
            ..path(color: const Color(0xf55f5f5f)),
        );

        // Finish gesture to release resources.
        await gesture.up();
        await tester.pumpAndSettle();
      } finally {
        debugDisableShadows = true;
      }
    });
  });
}

class RoundedRectSliderTrackShapeWithCustomAdditionalActiveTrackHeight
    extends RoundedRectSliderTrackShape {
  const RoundedRectSliderTrackShapeWithCustomAdditionalActiveTrackHeight({
    required this.additionalActiveTrackHeight,
  });
  final double additionalActiveTrackHeight;
  @override
  void paint(
    PaintingContext context,
    Offset offset, {
    required RenderBox parentBox,
    required SliderThemeData sliderTheme,
    required Animation<double> enableAnimation,
    required TextDirection textDirection,
    required Offset thumbCenter,
    Offset? secondaryOffset,
    bool isDiscrete = false,
    bool isEnabled = false,
    double additionalActiveTrackHeight = 2.0,
  }) {
    super.paint(
      context,
      offset,
      parentBox: parentBox,
      sliderTheme: sliderTheme,
      enableAnimation: enableAnimation,
      textDirection: textDirection,
      thumbCenter: thumbCenter,
      secondaryOffset: secondaryOffset,
      additionalActiveTrackHeight: this.additionalActiveTrackHeight,
    );
  }
}

Widget _buildApp(
  SliderThemeData sliderTheme, {
  double value = 0.0,
  double? secondaryTrackValue,
  bool enabled = true,
  int? divisions,
  FocusNode? focusNode,
}) {
  final ValueChanged<double>? onChanged = enabled ? (double d) => value = d : null;
  return MaterialApp(
    home: Scaffold(
      body: Center(
        child: SliderTheme(
          data: sliderTheme,
          child: Slider(
            value: value,
            secondaryTrackValue: secondaryTrackValue,
            label: '$value',
            onChanged: onChanged,
            divisions: divisions,
            focusNode: focusNode,
          ),
        ),
      ),
    ),
  );
}

Widget _buildRangeApp(
  SliderThemeData sliderTheme, {
  RangeValues values = const RangeValues(0, 0),
  bool enabled = true,
  int? divisions,
}) {
  final ValueChanged<RangeValues>? onChanged = enabled ? (RangeValues d) => values = d : null;
  return MaterialApp(
    home: Scaffold(
      body: Center(
        child: SliderTheme(
          data: sliderTheme,
          child: RangeSlider(
            values: values,
            labels: RangeLabels(values.start.toString(), values.end.toString()),
            onChanged: onChanged,
            divisions: divisions,
          ),
        ),
      ),
    ),
  );
}
