// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('TimePickerThemeData copyWith, ==, hashCode basics', () {
    expect(const TimePickerThemeData(), const TimePickerThemeData().copyWith());
    expect(const TimePickerThemeData().hashCode, const TimePickerThemeData().copyWith().hashCode);
  });

  test('TimePickerThemeData lerp special cases', () {
    const TimePickerThemeData data = TimePickerThemeData();
    expect(identical(TimePickerThemeData.lerp(data, data, 0.5), data), true);
  });

  test('TimePickerThemeData has null fields by default', () {
    const TimePickerThemeData timePickerTheme = TimePickerThemeData();
    expect(timePickerTheme.backgroundColor, null);
    expect(timePickerTheme.cancelButtonStyle, null);
    expect(timePickerTheme.confirmButtonStyle, null);
    expect(timePickerTheme.dayPeriodBorderSide, null);
    expect(timePickerTheme.dayPeriodColor, null);
    expect(timePickerTheme.dayPeriodShape, null);
    expect(timePickerTheme.dayPeriodTextColor, null);
    expect(timePickerTheme.dayPeriodTextStyle, null);
    expect(timePickerTheme.dialBackgroundColor, null);
    expect(timePickerTheme.dialHandColor, null);
    expect(timePickerTheme.dialTextColor, null);
    expect(timePickerTheme.dialTextStyle, null);
    expect(timePickerTheme.elevation, null);
    expect(timePickerTheme.entryModeIconColor, null);
    expect(timePickerTheme.helpTextStyle, null);
    expect(timePickerTheme.hourMinuteColor, null);
    expect(timePickerTheme.hourMinuteShape, null);
    expect(timePickerTheme.hourMinuteTextColor, null);
    expect(timePickerTheme.hourMinuteTextStyle, null);
    expect(timePickerTheme.inputDecorationTheme, null);
    expect(timePickerTheme.entryModeIconColor, null);
    expect(timePickerTheme.padding, null);
    expect(timePickerTheme.shape, null);
    expect(timePickerTheme.timeSelectorSeparatorColor, null);
    expect(timePickerTheme.timeSelectorSeparatorTextStyle, null);
  });

  testWidgets('Default TimePickerThemeData debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const TimePickerThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('TimePickerThemeData implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const TimePickerThemeData(
      backgroundColor: Color(0xfffffff0),
      cancelButtonStyle: ButtonStyle(
        foregroundColor: MaterialStatePropertyAll<Color>(Color(0xfffffff1)),
      ),
      confirmButtonStyle: ButtonStyle(
        foregroundColor: MaterialStatePropertyAll<Color>(Color(0xfffffff2)),
      ),
      dayPeriodBorderSide: BorderSide(color: Color(0xfffffff3)),
      dayPeriodColor: Color(0x00000000),
      dayPeriodShape: RoundedRectangleBorder(side: BorderSide(color: Color(0xfffffff5))),
      dayPeriodTextColor: Color(0xfffffff6),
      dayPeriodTextStyle: TextStyle(color: Color(0xfffffff7)),
      dialBackgroundColor: Color(0xfffffff8),
      dialHandColor: Color(0xfffffff9),
      dialTextColor: Color(0xfffffffa),
      dialTextStyle: TextStyle(color: Color(0xfffffffb)),
      elevation: 1.0,
      entryModeIconColor: Color(0xfffffffc),
      helpTextStyle: TextStyle(color: Color(0xfffffffd)),
      hourMinuteColor: Color(0xfffffffe),
      hourMinuteShape: RoundedRectangleBorder(side: BorderSide(color: Color(0xffffffff))),
      hourMinuteTextColor: Color(0xfffffff0),
      hourMinuteTextStyle: TextStyle(color: Color(0xfffffff1)),
      inputDecorationTheme: InputDecorationTheme(labelStyle: TextStyle(color: Color(0xfffffff2))),
      padding: EdgeInsets.all(1.0),
      shape: RoundedRectangleBorder(side: BorderSide(color: Color(0xfffffff3))),
      timeSelectorSeparatorColor: WidgetStatePropertyAll<Color>(Color(0xfffffff4)),
      timeSelectorSeparatorTextStyle: WidgetStatePropertyAll<TextStyle>(
        TextStyle(color: Color(0xfffffff5)),
      ),
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(
      description,
      equalsIgnoringHashCodes(<String>[
        'backgroundColor: ${const Color(0xfffffff0)}',
        'cancelButtonStyle: ButtonStyle#00000(foregroundColor: WidgetStatePropertyAll(${const Color(0xfffffff1)}))',
        'confirmButtonStyle: ButtonStyle#00000(foregroundColor: WidgetStatePropertyAll(${const Color(0xfffffff2)}))',
        'dayPeriodBorderSide: BorderSide(color: ${const Color(0xfffffff3)})',
        'dayPeriodColor: ${const Color(0x00000000)}',
        'dayPeriodShape: RoundedRectangleBorder(BorderSide(color: ${const Color(0xfffffff5)}), BorderRadius.zero)',
        'dayPeriodTextColor: ${const Color(0xfffffff6)}',
        'dayPeriodTextStyle: TextStyle(inherit: true, color: ${const Color(0xfffffff7)})',
        'dialBackgroundColor: ${const Color(0xfffffff8)}',
        'dialHandColor: ${const Color(0xfffffff9)}',
        'dialTextColor: ${const Color(0xfffffffa)}',
        'dialTextStyle: TextStyle(inherit: true, color: ${const Color(0xfffffffb)})',
        'elevation: 1.0',
        'entryModeIconColor: ${const Color(0xfffffffc)}',
        'helpTextStyle: TextStyle(inherit: true, color: ${const Color(0xfffffffd)})',
        'hourMinuteColor: ${const Color(0xfffffffe)}',
        'hourMinuteShape: RoundedRectangleBorder(BorderSide(color: ${const Color(0xffffffff)}), BorderRadius.zero)',
        'hourMinuteTextColor: ${const Color(0xfffffff0)}',
        'hourMinuteTextStyle: TextStyle(inherit: true, color: ${const Color(0xfffffff1)})',
        'inputDecorationTheme: InputDecorationTheme#ff861(labelStyle: TextStyle(inherit: true, color: ${const Color(0xfffffff2)}))',
        'padding: EdgeInsets.all(1.0)',
        'shape: RoundedRectangleBorder(BorderSide(color: ${const Color(0xfffffff3)}), BorderRadius.zero)',
        'timeSelectorSeparatorColor: WidgetStatePropertyAll(${const Color(0xfffffff4)})',
        'timeSelectorSeparatorTextStyle: WidgetStatePropertyAll(TextStyle(inherit: true, color: ${const Color(0xfffffff5)}))',
      ]),
    );
  });

  testWidgets('Material2 - Passing no TimePickerThemeData uses defaults', (
    WidgetTester tester,
  ) async {
    final ThemeData defaultTheme = ThemeData(useMaterial3: false);
    await tester.pumpWidget(_TimePickerLauncher(themeData: defaultTheme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle(const Duration(seconds: 1));

    final Material dialogMaterial = _dialogMaterial(tester);
    expect(dialogMaterial.color, defaultTheme.colorScheme.surface);
    expect(
      dialogMaterial.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4.0))),
    );

    final RenderBox dial = tester.firstRenderObject<RenderBox>(find.byType(CustomPaint));
    expect(
      dial,
      paints
        ..circle(
          color: defaultTheme.colorScheme.onSurface.withOpacity(0.08),
        ) // Dial background color.
        ..circle(color: Color(defaultTheme.colorScheme.primary.value)),
    );

    final RenderParagraph hourText = _textRenderParagraph(tester, '7');
    expect(
      hourText.text.style,
      Typography.material2014().englishLike.displayMedium!
          .merge(Typography.material2014().black.displayMedium)
          .copyWith(color: defaultTheme.colorScheme.primary),
    );

    final RenderParagraph minuteText = _textRenderParagraph(tester, '15');
    expect(
      minuteText.text.style,
      Typography.material2014().englishLike.displayMedium!
          .merge(Typography.material2014().black.displayMedium)
          .copyWith(color: defaultTheme.colorScheme.onSurface),
    );

    final RenderParagraph amText = _textRenderParagraph(tester, 'AM');
    expect(
      amText.text.style,
      Typography.material2014().englishLike.titleMedium!
          .merge(Typography.material2014().black.titleMedium)
          .copyWith(color: defaultTheme.colorScheme.primary),
    );

    final RenderParagraph pmText = _textRenderParagraph(tester, 'PM');
    expect(
      pmText.text.style,
      Typography.material2014().englishLike.titleMedium!
          .merge(Typography.material2014().black.titleMedium)
          .copyWith(color: defaultTheme.colorScheme.onSurface.withOpacity(0.6)),
    );

    final RenderParagraph helperText = _textRenderParagraph(tester, 'SELECT TIME');
    expect(
      helperText.text.style,
      Typography.material2014().englishLike.labelSmall!.merge(
        Typography.material2014().black.labelSmall,
      ),
    );

    final List<TappableLabel> primary, selected;
    (:primary, :selected) = TappableLabel.labels(tester.widget(findDialPaint));
    expect(
      primary.first.textSpan?.style,
      Typography.material2014().englishLike.bodyLarge!
          .merge(Typography.material2014().black.bodyLarge)
          .copyWith(color: defaultTheme.colorScheme.onSurface),
    );
    expect(
      selected.first.textSpan?.style,
      Typography.material2014().englishLike.bodyLarge!
          .merge(Typography.material2014().white.bodyLarge)
          .copyWith(color: defaultTheme.colorScheme.onPrimary),
    );

    final Material hourMaterial = _textMaterial(tester, '7');
    expect(hourMaterial.color, defaultTheme.colorScheme.primary.withOpacity(0.12));
    expect(
      hourMaterial.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4.0))),
    );

    final Material minuteMaterial = _textMaterial(tester, '15');
    expect(minuteMaterial.color, defaultTheme.colorScheme.onSurface.withOpacity(0.12));
    expect(
      minuteMaterial.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4.0))),
    );

    final Material amMaterial = _textMaterial(tester, 'AM');
    expect(amMaterial.color, defaultTheme.colorScheme.primary.withOpacity(0.12));

    final Material pmMaterial = _textMaterial(tester, 'PM');
    expect(pmMaterial.color, Colors.transparent);

    final Color expectedBorderColor = Color.alphaBlend(
      defaultTheme.colorScheme.onSurface.withOpacity(0.38),
      defaultTheme.colorScheme.surface,
    );
    final Material dayPeriodMaterial = _dayPeriodMaterial(tester);
    expect(
      dayPeriodMaterial.shape,
      RoundedRectangleBorder(
        borderRadius: const BorderRadius.all(Radius.circular(4.0)),
        side: BorderSide(color: expectedBorderColor),
      ),
    );

    final Container dayPeriodDivider = _dayPeriodDivider(tester);
    expect(
      dayPeriodDivider.decoration,
      BoxDecoration(border: Border(left: BorderSide(color: expectedBorderColor))),
    );

    final IconButton entryModeIconButton = _entryModeIconButton(tester);
    expect(entryModeIconButton.color, defaultTheme.colorScheme.onSurface.withOpacity(0.6));

    final ButtonStyle cancelButtonStyle = _actionButtonStyle(tester, 'CANCEL');
    expect(
      cancelButtonStyle.toString(),
      equalsIgnoringHashCodes(TextButton.styleFrom().toString()),
    );

    final ButtonStyle confirmButtonStyle = _actionButtonStyle(tester, 'OK');
    expect(
      confirmButtonStyle.toString(),
      equalsIgnoringHashCodes(TextButton.styleFrom().toString()),
    );
  });

  testWidgets('Material3 - Passing no TimePickerThemeData uses defaults', (
    WidgetTester tester,
  ) async {
    final ThemeData defaultTheme = ThemeData(useMaterial3: true);
    await tester.pumpWidget(_TimePickerLauncher(themeData: defaultTheme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle(const Duration(seconds: 1));

    final Material dialogMaterial = _dialogMaterial(tester);
    expect(dialogMaterial.color, defaultTheme.colorScheme.surfaceContainerHigh);
    expect(
      dialogMaterial.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(28.0))),
    );

    final RenderBox dial = tester.firstRenderObject<RenderBox>(find.byType(CustomPaint));
    expect(
      dial,
      paints
        ..circle(color: defaultTheme.colorScheme.surfaceContainerHighest) // Dial background color.
        ..circle(color: Color(defaultTheme.colorScheme.primary.value)), // Dial hand color.
    );

    final RenderParagraph hourText = _textRenderParagraph(tester, '7');
    expect(
      hourText.text.style,
      Typography.material2021().englishLike.displayLarge!
          .merge(Typography.material2021().black.displayLarge)
          .copyWith(
            color: defaultTheme.colorScheme.onPrimaryContainer,
            decorationColor: defaultTheme.colorScheme.onSurface,
          ),
    );

    final RenderParagraph minuteText = _textRenderParagraph(tester, '15');
    expect(
      minuteText.text.style,
      Typography.material2021().englishLike.displayLarge!
          .merge(Typography.material2021().black.displayLarge)
          .copyWith(
            color: defaultTheme.colorScheme.onSurface,
            decorationColor: defaultTheme.colorScheme.onSurface,
          ),
    );

    final RenderParagraph amText = _textRenderParagraph(tester, 'AM');
    expect(
      amText.text.style,
      Typography.material2021().englishLike.titleMedium!
          .merge(Typography.material2021().black.titleMedium)
          .copyWith(
            color: defaultTheme.colorScheme.onTertiaryContainer,
            decorationColor: defaultTheme.colorScheme.onSurface,
          ),
    );

    final RenderParagraph pmText = _textRenderParagraph(tester, 'PM');
    expect(
      pmText.text.style,
      Typography.material2021().englishLike.titleMedium!
          .merge(Typography.material2021().black.titleMedium)
          .copyWith(
            color: defaultTheme.colorScheme.onSurfaceVariant,
            decorationColor: defaultTheme.colorScheme.onSurface,
          ),
    );

    final RenderParagraph helperText = _textRenderParagraph(tester, 'Select time');
    expect(
      helperText.text.style,
      Typography.material2021().englishLike.bodyMedium!
          .merge(Typography.material2021().black.bodyMedium)
          .copyWith(
            color: defaultTheme.colorScheme.onSurface,
            decorationColor: defaultTheme.colorScheme.onSurface,
          ),
    );

    final List<TappableLabel> primary, selected;
    (:primary, :selected) = TappableLabel.labels(tester.widget(findDialPaint));
    expect(
      primary.first.textSpan?.style,
      Typography.material2021().englishLike.bodyLarge!
          .merge(Typography.material2021().black.bodyLarge)
          .copyWith(
            color: defaultTheme.colorScheme.onSurface,
            decorationColor: defaultTheme.colorScheme.onSurface,
          ),
    );
    expect(
      selected.first.textSpan?.style,
      Typography.material2021().englishLike.bodyLarge!
          .merge(Typography.material2021().black.bodyLarge)
          .copyWith(
            color: defaultTheme.colorScheme.onPrimary,
            decorationColor: defaultTheme.colorScheme.onSurface,
          ),
    );

    final Material hourMaterial = _textMaterial(tester, '7');
    expect(hourMaterial.color, defaultTheme.colorScheme.primaryContainer);
    expect(
      hourMaterial.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(8.0))),
    );

    final Material minuteMaterial = _textMaterial(tester, '15');
    expect(minuteMaterial.color, defaultTheme.colorScheme.surfaceContainerHighest);
    expect(
      minuteMaterial.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(8.0))),
    );

    final Material amMaterial = _textMaterial(tester, 'AM');
    expect(amMaterial.color, defaultTheme.colorScheme.tertiaryContainer);

    final Material pmMaterial = _textMaterial(tester, 'PM');
    expect(pmMaterial.color, Colors.transparent);

    final Material dayPeriodMaterial = _dayPeriodMaterial(tester);
    expect(
      dayPeriodMaterial.shape,
      RoundedRectangleBorder(
        borderRadius: const BorderRadius.all(Radius.circular(8.0)),
        side: BorderSide(color: defaultTheme.colorScheme.outline),
      ),
    );

    final Container dayPeriodDivider = _dayPeriodDivider(tester);
    expect(
      dayPeriodDivider.decoration,
      BoxDecoration(border: Border(left: BorderSide(color: defaultTheme.colorScheme.outline))),
    );

    final IconButton entryModeIconButton = _entryModeIconButton(tester);
    expect(entryModeIconButton.color, null);

    final ButtonStyle cancelButtonStyle = _actionButtonStyle(tester, 'Cancel');
    expect(
      cancelButtonStyle.toString(),
      equalsIgnoringHashCodes(TextButton.styleFrom().toString()),
    );

    final ButtonStyle confirmButtonStyle = _actionButtonStyle(tester, 'OK');
    expect(
      confirmButtonStyle.toString(),
      equalsIgnoringHashCodes(TextButton.styleFrom().toString()),
    );
  });

  testWidgets('Material2 - Passing no TimePickerThemeData uses defaults - input mode', (
    WidgetTester tester,
  ) async {
    final ThemeData defaultTheme = ThemeData(useMaterial3: false);
    await tester.pumpWidget(
      _TimePickerLauncher(themeData: defaultTheme, entryMode: TimePickerEntryMode.input),
    );
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle(const Duration(seconds: 1));

    final InputDecoration hourDecoration = _textField(tester, '7').decoration!;
    expect(hourDecoration.filled, true);
    expect(
      hourDecoration.fillColor,
      MaterialStateColor.resolveWith(
        (Set<MaterialState> states) => defaultTheme.colorScheme.onSurface.withOpacity(0.12),
      ),
    );
    expect(
      hourDecoration.enabledBorder,
      const OutlineInputBorder(borderSide: BorderSide(color: Colors.transparent)),
    );
    expect(
      hourDecoration.errorBorder,
      OutlineInputBorder(borderSide: BorderSide(color: defaultTheme.colorScheme.error, width: 2)),
    );
    expect(
      hourDecoration.focusedBorder,
      OutlineInputBorder(borderSide: BorderSide(color: defaultTheme.colorScheme.primary, width: 2)),
    );
    expect(
      hourDecoration.focusedErrorBorder,
      OutlineInputBorder(borderSide: BorderSide(color: defaultTheme.colorScheme.error, width: 2)),
    );
    expect(
      hourDecoration.hintStyle,
      Typography.material2014().englishLike.displayMedium!.merge(
        defaultTheme.textTheme.displayMedium!.copyWith(
          color: defaultTheme.colorScheme.onSurface.withOpacity(0.36),
        ),
      ),
    );

    final ButtonStyle cancelButtonStyle = _actionButtonStyle(tester, 'CANCEL');
    expect(
      cancelButtonStyle.toString(),
      equalsIgnoringHashCodes(TextButton.styleFrom().toString()),
    );

    final ButtonStyle confirmButtonStyle = _actionButtonStyle(tester, 'OK');
    expect(
      confirmButtonStyle.toString(),
      equalsIgnoringHashCodes(TextButton.styleFrom().toString()),
    );
  });

  testWidgets('Material3 - Passing no TimePickerThemeData uses defaults - input mode', (
    WidgetTester tester,
  ) async {
    final ThemeData defaultTheme = ThemeData(useMaterial3: true);
    await tester.pumpWidget(
      _TimePickerLauncher(themeData: defaultTheme, entryMode: TimePickerEntryMode.input),
    );
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle(const Duration(seconds: 1));

    final TextStyle hourTextStyle = _textField(tester, '7').style!;
    expect(
      hourTextStyle,
      Typography.material2021().englishLike.displayMedium!
          .merge(Typography.material2021().black.displayMedium)
          .copyWith(
            color: defaultTheme.colorScheme.onSurface,
            decorationColor: defaultTheme.colorScheme.onSurface,
          ),
    );

    final TextStyle minuteTextStyle = _textField(tester, '15').style!;
    expect(
      minuteTextStyle,
      Typography.material2021().englishLike.displayMedium!
          .merge(Typography.material2021().black.displayMedium)
          .copyWith(
            color: defaultTheme.colorScheme.onSurface,
            decorationColor: defaultTheme.colorScheme.onSurface,
          ),
    );

    final InputDecoration hourDecoration = _textField(tester, '7').decoration!;
    expect(hourDecoration.filled, true);
    expect(hourDecoration.fillColor, defaultTheme.colorScheme.surfaceContainerHighest);
    expect(
      hourDecoration.enabledBorder,
      const OutlineInputBorder(
        borderRadius: BorderRadius.all(Radius.circular(8.0)),
        borderSide: BorderSide(color: Colors.transparent),
      ),
    );
    expect(
      hourDecoration.errorBorder,
      OutlineInputBorder(
        borderRadius: const BorderRadius.all(Radius.circular(8.0)),
        borderSide: BorderSide(color: defaultTheme.colorScheme.error, width: 2.0),
      ),
    );
    expect(
      hourDecoration.focusedBorder,
      OutlineInputBorder(
        borderRadius: const BorderRadius.all(Radius.circular(8.0)),
        borderSide: BorderSide(color: defaultTheme.colorScheme.primary, width: 2.0),
      ),
    );
    expect(
      hourDecoration.focusedErrorBorder,
      OutlineInputBorder(
        borderRadius: const BorderRadius.all(Radius.circular(8.0)),
        borderSide: BorderSide(color: defaultTheme.colorScheme.error, width: 2.0),
      ),
    );
    expect(
      hourDecoration.hintStyle,
      TextStyle(color: defaultTheme.colorScheme.onSurface.withOpacity(0.36)),
    );

    final ButtonStyle cancelButtonStyle = _actionButtonStyle(tester, 'Cancel');
    expect(
      cancelButtonStyle.toString(),
      equalsIgnoringHashCodes(TextButton.styleFrom().toString()),
    );

    final ButtonStyle confirmButtonStyle = _actionButtonStyle(tester, 'OK');
    expect(
      confirmButtonStyle.toString(),
      equalsIgnoringHashCodes(TextButton.styleFrom().toString()),
    );
  });

  testWidgets('Material2 - Time picker uses values from TimePickerThemeData', (
    WidgetTester tester,
  ) async {
    final TimePickerThemeData timePickerTheme = _timePickerTheme();
    final ThemeData theme = ThemeData(timePickerTheme: timePickerTheme, useMaterial3: false);
    await tester.pumpWidget(_TimePickerLauncher(themeData: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle(const Duration(seconds: 1));

    final Material dialogMaterial = _dialogMaterial(tester);
    expect(dialogMaterial.color, timePickerTheme.backgroundColor);
    expect(dialogMaterial.shape, timePickerTheme.shape);

    final RenderBox dial = tester.firstRenderObject<RenderBox>(find.byType(CustomPaint));
    expect(
      dial,
      paints
        ..circle(color: Color(timePickerTheme.dialBackgroundColor!.value)) // Dial background color.
        ..circle(color: Color(timePickerTheme.dialHandColor!.value)), // Dial hand color.
    );

    final RenderParagraph hourText = _textRenderParagraph(tester, '7');
    expect(
      hourText.text.style,
      Typography.material2014().englishLike.bodyMedium!
          .merge(Typography.material2014().black.bodyMedium)
          .merge(timePickerTheme.hourMinuteTextStyle)
          .copyWith(color: _selectedColor),
    );

    final RenderParagraph minuteText = _textRenderParagraph(tester, '15');
    expect(
      minuteText.text.style,
      Typography.material2014().englishLike.bodyMedium!
          .merge(Typography.material2014().black.bodyMedium)
          .merge(timePickerTheme.hourMinuteTextStyle)
          .copyWith(color: _unselectedColor),
    );

    final RenderParagraph amText = _textRenderParagraph(tester, 'AM');
    expect(
      amText.text.style,
      Typography.material2014().englishLike.titleMedium!
          .merge(Typography.material2014().black.titleMedium)
          .merge(timePickerTheme.dayPeriodTextStyle)
          .copyWith(color: _selectedColor),
    );

    final RenderParagraph pmText = _textRenderParagraph(tester, 'PM');
    expect(
      pmText.text.style,
      Typography.material2014().englishLike.titleMedium!
          .merge(Typography.material2014().black.titleMedium)
          .merge(timePickerTheme.dayPeriodTextStyle)
          .copyWith(color: _unselectedColor),
    );

    final RenderParagraph helperText = _textRenderParagraph(tester, 'SELECT TIME');
    expect(
      helperText.text.style,
      Typography.material2014().englishLike.bodyMedium!
          .merge(Typography.material2014().black.bodyMedium)
          .merge(timePickerTheme.helpTextStyle),
    );

    final List<TappableLabel> primary, selected;
    (:primary, :selected) = TappableLabel.labels(tester.widget(findDialPaint));
    expect(
      primary.first.textSpan?.style,
      Typography.material2014().englishLike.bodyLarge!
          .merge(Typography.material2014().black.bodyLarge)
          .copyWith(color: _unselectedColor),
    );
    expect(
      selected.first.textSpan?.style,
      Typography.material2014().englishLike.bodyLarge!
          .merge(Typography.material2014().white.bodyLarge)
          .copyWith(color: _selectedColor),
    );

    final Material hourMaterial = _textMaterial(tester, '7');
    expect(hourMaterial.color, _selectedColor);
    expect(hourMaterial.shape, timePickerTheme.hourMinuteShape);

    final Material minuteMaterial = _textMaterial(tester, '15');
    expect(minuteMaterial.color, _unselectedColor);
    expect(minuteMaterial.shape, timePickerTheme.hourMinuteShape);

    final Material amMaterial = _textMaterial(tester, 'AM');
    expect(amMaterial.color, _selectedColor);

    final Material pmMaterial = _textMaterial(tester, 'PM');
    expect(pmMaterial.color, _unselectedColor);

    final Material dayPeriodMaterial = _dayPeriodMaterial(tester);
    expect(
      dayPeriodMaterial.shape,
      timePickerTheme.dayPeriodShape!.copyWith(side: timePickerTheme.dayPeriodBorderSide),
    );

    final Container dayPeriodDivider = _dayPeriodDivider(tester);
    expect(
      dayPeriodDivider.decoration,
      BoxDecoration(border: Border(left: timePickerTheme.dayPeriodBorderSide!)),
    );

    final IconButton entryModeIconButton = _entryModeIconButton(tester);
    expect(entryModeIconButton.color, timePickerTheme.entryModeIconColor);

    final ButtonStyle cancelButtonStyle = _actionButtonStyle(tester, 'CANCEL');
    expect(
      cancelButtonStyle.toString(),
      equalsIgnoringHashCodes(timePickerTheme.cancelButtonStyle.toString()),
    );

    final ButtonStyle confirmButtonStyle = _actionButtonStyle(tester, 'OK');
    expect(
      confirmButtonStyle.toString(),
      equalsIgnoringHashCodes(timePickerTheme.confirmButtonStyle.toString()),
    );
  });

  testWidgets('Material3 - Time picker uses values from TimePickerThemeData', (
    WidgetTester tester,
  ) async {
    final TimePickerThemeData timePickerTheme = _timePickerTheme();
    final ThemeData theme = ThemeData(timePickerTheme: timePickerTheme, useMaterial3: true);
    await tester.pumpWidget(_TimePickerLauncher(themeData: theme));
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle(const Duration(seconds: 1));

    final Material dialogMaterial = _dialogMaterial(tester);
    expect(dialogMaterial.color, timePickerTheme.backgroundColor);
    expect(dialogMaterial.shape, timePickerTheme.shape);

    final RenderBox dial = tester.firstRenderObject<RenderBox>(find.byType(CustomPaint));
    expect(
      dial,
      paints
        ..circle(color: Color(timePickerTheme.dialBackgroundColor!.value)) // Dial background color.
        ..circle(color: Color(timePickerTheme.dialHandColor!.value)), // Dial hand color.
    );

    final RenderParagraph hourText = _textRenderParagraph(tester, '7');
    expect(
      hourText.text.style,
      Typography.material2021().englishLike.bodyMedium!
          .merge(Typography.material2021().black.bodyMedium)
          .merge(timePickerTheme.hourMinuteTextStyle)
          .copyWith(color: _selectedColor, decorationColor: const Color(0xff1d1b20)),
    );

    final RenderParagraph minuteText = _textRenderParagraph(tester, '15');
    expect(
      minuteText.text.style,
      Typography.material2021().englishLike.bodyMedium!
          .merge(Typography.material2021().black.bodyMedium)
          .merge(timePickerTheme.hourMinuteTextStyle)
          .copyWith(color: _unselectedColor, decorationColor: const Color(0xff1d1b20)),
    );

    final RenderParagraph amText = _textRenderParagraph(tester, 'AM');
    expect(
      amText.text.style,
      Typography.material2021().englishLike.bodyMedium!
          .merge(Typography.material2021().black.bodyMedium)
          .merge(timePickerTheme.hourMinuteTextStyle)
          .copyWith(color: _selectedColor, decorationColor: const Color(0xff1d1b20)),
    );

    final RenderParagraph pmText = _textRenderParagraph(tester, 'PM');
    expect(
      pmText.text.style,
      Typography.material2021().englishLike.bodyMedium!
          .merge(Typography.material2021().black.bodyMedium)
          .merge(timePickerTheme.hourMinuteTextStyle)
          .copyWith(color: _unselectedColor, decorationColor: const Color(0xff1d1b20)),
    );

    final RenderParagraph helperText = _textRenderParagraph(tester, 'Select time');
    expect(
      helperText.text.style,
      Typography.material2021().englishLike.bodyMedium!
          .merge(Typography.material2021().black.bodyMedium)
          .merge(timePickerTheme.helpTextStyle)
          .copyWith(
            color: theme.colorScheme.onSurface,
            decorationColor: theme.colorScheme.onSurface,
          ),
    );

    final List<TappableLabel> primary, selected;
    (:primary, :selected) = TappableLabel.labels(tester.widget(findDialPaint));
    expect(
      primary.first.textSpan?.style,
      Typography.material2021().englishLike.bodyLarge!
          .merge(Typography.material2021().black.bodyLarge)
          .copyWith(color: _unselectedColor, decorationColor: theme.colorScheme.onSurface),
    );
    expect(
      selected.first.textSpan?.style,
      Typography.material2021().englishLike.bodyLarge!
          .merge(Typography.material2021().black.bodyLarge)
          .copyWith(color: _selectedColor, decorationColor: theme.colorScheme.onSurface),
    );

    final Material hourMaterial = _textMaterial(tester, '7');
    expect(hourMaterial.color, _selectedColor);
    expect(hourMaterial.shape, timePickerTheme.hourMinuteShape);

    final Material minuteMaterial = _textMaterial(tester, '15');
    expect(minuteMaterial.color, _unselectedColor);
    expect(minuteMaterial.shape, timePickerTheme.hourMinuteShape);

    final Material amMaterial = _textMaterial(tester, 'AM');
    expect(amMaterial.color, _selectedColor);

    final Material pmMaterial = _textMaterial(tester, 'PM');
    expect(pmMaterial.color, _unselectedColor);

    final Material dayPeriodMaterial = _dayPeriodMaterial(tester);
    expect(
      dayPeriodMaterial.shape,
      timePickerTheme.dayPeriodShape!.copyWith(side: timePickerTheme.dayPeriodBorderSide),
    );

    final Container dayPeriodDivider = _dayPeriodDivider(tester);
    expect(
      dayPeriodDivider.decoration,
      BoxDecoration(border: Border(left: timePickerTheme.dayPeriodBorderSide!)),
    );

    final IconButton entryModeIconButton = _entryModeIconButton(tester);
    expect(entryModeIconButton.color, null);

    final ButtonStyle cancelButtonStyle = _actionButtonStyle(tester, 'Cancel');
    expect(
      cancelButtonStyle.toString(),
      equalsIgnoringHashCodes(timePickerTheme.cancelButtonStyle.toString()),
    );

    final ButtonStyle confirmButtonStyle = _actionButtonStyle(tester, 'OK');
    expect(
      confirmButtonStyle.toString(),
      equalsIgnoringHashCodes(timePickerTheme.confirmButtonStyle.toString()),
    );
  });

  testWidgets(
    'Time picker uses values from TimePickerThemeData with InputDecorationTheme - input mode',
    (WidgetTester tester) async {
      final TimePickerThemeData timePickerTheme = _timePickerTheme(includeInputDecoration: true);
      final ThemeData theme = ThemeData(timePickerTheme: timePickerTheme);
      await tester.pumpWidget(
        _TimePickerLauncher(themeData: theme, entryMode: TimePickerEntryMode.input),
      );
      await tester.tap(find.text('X'));
      await tester.pumpAndSettle(const Duration(seconds: 1));

      final InputDecoration hourDecoration = _textField(tester, '7').decoration!;
      expect(hourDecoration.filled, timePickerTheme.inputDecorationTheme!.filled);
      expect(hourDecoration.fillColor, timePickerTheme.inputDecorationTheme!.fillColor);
      expect(hourDecoration.enabledBorder, timePickerTheme.inputDecorationTheme!.enabledBorder);
      expect(hourDecoration.errorBorder, timePickerTheme.inputDecorationTheme!.errorBorder);
      expect(hourDecoration.focusedBorder, timePickerTheme.inputDecorationTheme!.focusedBorder);
      expect(
        hourDecoration.focusedErrorBorder,
        timePickerTheme.inputDecorationTheme!.focusedErrorBorder,
      );
      expect(hourDecoration.hintStyle, timePickerTheme.inputDecorationTheme!.hintStyle);
    },
  );

  testWidgets(
    'Time picker uses values from TimePickerThemeData without InputDecorationTheme - input mode',
    (WidgetTester tester) async {
      final TimePickerThemeData timePickerTheme = _timePickerTheme();
      final ThemeData theme = ThemeData(timePickerTheme: timePickerTheme);
      await tester.pumpWidget(
        _TimePickerLauncher(themeData: theme, entryMode: TimePickerEntryMode.input),
      );
      await tester.tap(find.text('X'));
      await tester.pumpAndSettle(const Duration(seconds: 1));

      final InputDecoration hourDecoration = _textField(tester, '7').decoration!;
      expect(hourDecoration.fillColor?.value, timePickerTheme.hourMinuteColor?.value);
    },
  );

  testWidgets('Time picker dayPeriodColor does the right thing with non-MaterialStateColor', (
    WidgetTester tester,
  ) async {
    final TimePickerThemeData timePickerTheme = _timePickerTheme().copyWith(
      dayPeriodColor: Colors.red,
    );
    final ThemeData theme = ThemeData(timePickerTheme: timePickerTheme);
    await tester.pumpWidget(
      _TimePickerLauncher(themeData: theme, entryMode: TimePickerEntryMode.input),
    );
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle(const Duration(seconds: 1));

    final Material amMaterial = _textMaterial(tester, 'AM');
    expect(amMaterial.color, Colors.red);

    final Material pmMaterial = _textMaterial(tester, 'PM');
    expect(pmMaterial.color, Colors.transparent);
  });

  testWidgets('Time picker dayPeriodColor does the right thing with MaterialStateColor', (
    WidgetTester tester,
  ) async {
    final MaterialStateColor testColor = MaterialStateColor.resolveWith((
      Set<MaterialState> states,
    ) {
      if (states.contains(MaterialState.selected)) {
        return Colors.green;
      }
      return Colors.blue;
    });

    final TimePickerThemeData timePickerTheme = _timePickerTheme().copyWith(
      dayPeriodColor: testColor,
    );
    final ThemeData theme = ThemeData(timePickerTheme: timePickerTheme);
    await tester.pumpWidget(
      _TimePickerLauncher(themeData: theme, entryMode: TimePickerEntryMode.input),
    );
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle(const Duration(seconds: 1));

    final Material amMaterial = _textMaterial(tester, 'AM');
    expect(amMaterial.color, Colors.green);

    final Material pmMaterial = _textMaterial(tester, 'PM');
    expect(pmMaterial.color, Colors.blue);
  });

  testWidgets('Time selector separator color uses the timeSelectorSeparatorColor value', (
    WidgetTester tester,
  ) async {
    final TimePickerThemeData timePickerTheme = _timePickerTheme().copyWith(
      timeSelectorSeparatorColor: const MaterialStatePropertyAll<Color>(Color(0xff00ff00)),
    );
    final ThemeData theme = ThemeData(timePickerTheme: timePickerTheme);
    await tester.pumpWidget(
      _TimePickerLauncher(themeData: theme, entryMode: TimePickerEntryMode.input),
    );
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle(const Duration(seconds: 1));

    final RenderParagraph paragraph = tester.renderObject(find.text(':'));
    expect(paragraph.text.style!.color, const Color(0xff00ff00));
  });

  testWidgets('Time selector separator text style uses the timeSelectorSeparatorTextStyle value', (
    WidgetTester tester,
  ) async {
    final TimePickerThemeData timePickerTheme = _timePickerTheme().copyWith(
      timeSelectorSeparatorTextStyle: const MaterialStatePropertyAll<TextStyle>(
        TextStyle(fontSize: 35.0, fontStyle: FontStyle.italic),
      ),
    );
    final ThemeData theme = ThemeData(timePickerTheme: timePickerTheme);
    await tester.pumpWidget(
      _TimePickerLauncher(themeData: theme, entryMode: TimePickerEntryMode.input),
    );
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle(const Duration(seconds: 1));

    final RenderParagraph paragraph = tester.renderObject(find.text(':'));
    expect(paragraph.text.style!.fontSize, 35.0);
    expect(paragraph.text.style!.fontStyle, FontStyle.italic);
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/153549.
  testWidgets('Time picker hour minute does not resize on error', (WidgetTester tester) async {
    final TimePickerThemeData timePickerTheme = _timePickerTheme(includeInputDecoration: true);
    final ThemeData theme = ThemeData(timePickerTheme: timePickerTheme);
    await tester.pumpWidget(
      _TimePickerLauncher(themeData: theme, entryMode: TimePickerEntryMode.input),
    );
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle(const Duration(seconds: 1));

    expect(tester.getSize(findBorderPainter().first), const Size(96.0, 72.0));

    // Enter invalid hour.
    await tester.enterText(find.byType(TextField).first, 'AB');
    await tester.tap(find.text('OK'));

    expect(tester.getSize(findBorderPainter().first), const Size(96.0, 72.0));
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/153549.
  testWidgets('Material2 - Time picker hour minute does not resize on error', (
    WidgetTester tester,
  ) async {
    final TimePickerThemeData timePickerTheme = _timePickerTheme(includeInputDecoration: true);
    final ThemeData theme = ThemeData(timePickerTheme: timePickerTheme, useMaterial3: false);
    await tester.pumpWidget(
      _TimePickerLauncher(themeData: theme, entryMode: TimePickerEntryMode.input),
    );
    await tester.tap(find.text('X'));
    await tester.pumpAndSettle(const Duration(seconds: 1));

    expect(tester.getSize(findBorderPainter().first), const Size(96.0, 70.0));

    // Enter invalid hour.
    await tester.enterText(find.byType(TextField).first, 'AB');
    await tester.tap(find.text('OK'));

    expect(tester.getSize(findBorderPainter().first), const Size(96.0, 70.0));
  });
}

final Color _selectedColor = Colors.green[100]!;
final Color _unselectedColor = Colors.green[200]!;

TimePickerThemeData _timePickerTheme({bool includeInputDecoration = false}) {
  Color getColor(Set<MaterialState> states) {
    return states.contains(MaterialState.selected) ? _selectedColor : _unselectedColor;
  }

  final MaterialStateColor materialStateColor = MaterialStateColor.resolveWith(getColor);
  return TimePickerThemeData(
    backgroundColor: Colors.orange,
    cancelButtonStyle: TextButton.styleFrom(foregroundColor: Colors.red),
    confirmButtonStyle: TextButton.styleFrom(foregroundColor: Colors.green),
    hourMinuteTextColor: materialStateColor,
    hourMinuteColor: materialStateColor,
    dayPeriodTextColor: materialStateColor,
    dayPeriodColor: materialStateColor,
    dialHandColor: Colors.brown,
    dialBackgroundColor: Colors.pinkAccent,
    dialTextColor: materialStateColor,
    entryModeIconColor: Colors.red,
    hourMinuteTextStyle: const TextStyle(fontSize: 8.0),
    dayPeriodTextStyle: const TextStyle(fontSize: 8.0),
    helpTextStyle: const TextStyle(fontSize: 8.0),
    shape: const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(16.0))),
    hourMinuteShape: const RoundedRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(16.0)),
    ),
    dayPeriodShape: const RoundedRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(16.0)),
    ),
    dayPeriodBorderSide: const BorderSide(color: Colors.blueAccent),
    inputDecorationTheme:
        includeInputDecoration
            ? const InputDecorationTheme(
              filled: true,
              fillColor: Colors.purple,
              enabledBorder: OutlineInputBorder(borderSide: BorderSide(color: Colors.blue)),
              errorBorder: OutlineInputBorder(borderSide: BorderSide(color: Colors.green)),
              focusedBorder: OutlineInputBorder(borderSide: BorderSide(color: Colors.yellow)),
              focusedErrorBorder: OutlineInputBorder(borderSide: BorderSide(color: Colors.red)),
              hintStyle: TextStyle(fontSize: 8),
            )
            : null,
  );
}

class _TimePickerLauncher extends StatelessWidget {
  const _TimePickerLauncher({this.themeData, this.entryMode = TimePickerEntryMode.dial});

  final ThemeData? themeData;
  final TimePickerEntryMode entryMode;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: themeData,
      home: Material(
        child: Center(
          child: Builder(
            builder: (BuildContext context) {
              return ElevatedButton(
                child: const Text('X'),
                onPressed: () async {
                  await showTimePicker(
                    context: context,
                    initialEntryMode: entryMode,
                    initialTime: const TimeOfDay(hour: 7, minute: 15),
                  );
                },
              );
            },
          ),
        ),
      ),
    );
  }
}

Material _dialogMaterial(WidgetTester tester) {
  return tester.widget<Material>(
    find.descendant(of: find.byType(Dialog), matching: find.byType(Material)).first,
  );
}

Material _textMaterial(WidgetTester tester, String text) {
  return tester.widget<Material>(
    find.ancestor(of: find.text(text), matching: find.byType(Material)).first,
  );
}

TextField _textField(WidgetTester tester, String text) {
  return tester.widget<TextField>(
    find.ancestor(of: find.text(text), matching: find.byType(TextField)).first,
  );
}

Material _dayPeriodMaterial(WidgetTester tester) {
  return tester.widget<Material>(
    find
        .descendant(
          of: find.byWidgetPredicate((Widget w) => '${w.runtimeType}' == '_DayPeriodControl'),
          matching: find.byType(Material),
        )
        .first,
  );
}

Container _dayPeriodDivider(WidgetTester tester) {
  return tester.widget<Container>(
    find
        .descendant(
          of: find.byWidgetPredicate((Widget w) => '${w.runtimeType}' == '_DayPeriodControl'),
          matching: find.byType(Container),
        )
        .at(0),
  );
}

IconButton _entryModeIconButton(WidgetTester tester) {
  return tester.widget<IconButton>(
    find.descendant(of: find.byType(Dialog), matching: find.byType(IconButton)).first,
  );
}

RenderParagraph _textRenderParagraph(WidgetTester tester, String text) {
  return tester.element<StatelessElement>(find.text(text).first).renderObject! as RenderParagraph;
}

final Finder findDialPaint = find.descendant(
  of: find.byWidgetPredicate((Widget w) => '${w.runtimeType}' == '_Dial'),
  matching: find.byWidgetPredicate((Widget w) => w is CustomPaint),
);

ButtonStyle _actionButtonStyle(WidgetTester tester, String text) {
  return tester.widget<TextButton>(find.widgetWithText(TextButton, text)).style!;
}

Finder findBorderPainter() {
  return find.descendant(
    of: find.byWidgetPredicate((Widget w) => '${w.runtimeType}' == '_BorderContainer'),
    matching: find.byWidgetPredicate((Widget w) => w is CustomPaint),
  );
}
