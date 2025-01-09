// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  const DatePickerThemeData datePickerTheme = DatePickerThemeData(
    backgroundColor: Color(0xfffffff0),
    elevation: 6,
    shadowColor: Color(0xfffffff1),
    surfaceTintColor: Color(0xfffffff2),
    shape: RoundedRectangleBorder(),
    headerBackgroundColor: Color(0xfffffff3),
    headerForegroundColor: Color(0xfffffff4),
    headerHeadlineStyle: TextStyle(fontSize: 10),
    headerHelpStyle: TextStyle(fontSize: 11),
    weekdayStyle: TextStyle(fontSize: 12),
    dayStyle: TextStyle(fontSize: 13),
    dayForegroundColor: MaterialStatePropertyAll<Color>(Color(0xfffffff5)),
    dayBackgroundColor: MaterialStatePropertyAll<Color>(Color(0xfffffff6)),
    dayOverlayColor: MaterialStatePropertyAll<Color>(Color(0xfffffff7)),
    dayShape: MaterialStatePropertyAll<OutlinedBorder>(RoundedRectangleBorder()),
    todayForegroundColor: MaterialStatePropertyAll<Color>(Color(0xfffffff8)),
    todayBackgroundColor: MaterialStatePropertyAll<Color>(Color(0xfffffff9)),
    todayBorder: BorderSide(width: 3),
    yearStyle: TextStyle(fontSize: 13),
    yearForegroundColor: MaterialStatePropertyAll<Color>(Color(0xfffffffa)),
    yearBackgroundColor: MaterialStatePropertyAll<Color>(Color(0xfffffffb)),
    yearOverlayColor: MaterialStatePropertyAll<Color>(Color(0xfffffffc)),
    rangePickerBackgroundColor: Color(0xfffffffd),
    rangePickerElevation: 7,
    rangePickerShadowColor: Color(0xfffffffe),
    rangePickerSurfaceTintColor: Color(0xffffffff),
    rangePickerShape: RoundedRectangleBorder(),
    rangePickerHeaderBackgroundColor: Color(0xffffff0f),
    rangePickerHeaderForegroundColor: Color(0xffffff1f),
    rangePickerHeaderHeadlineStyle: TextStyle(fontSize: 14),
    rangePickerHeaderHelpStyle: TextStyle(fontSize: 15),
    rangeSelectionBackgroundColor: Color(0xffffff2f),
    rangeSelectionOverlayColor: MaterialStatePropertyAll<Color>(Color(0xffffff3f)),
    dividerColor: Color(0xffffff4f),
    inputDecorationTheme: InputDecorationTheme(
      fillColor: Color(0xffffff5f),
      border: UnderlineInputBorder(),
    ),
    cancelButtonStyle: ButtonStyle(
      foregroundColor: MaterialStatePropertyAll<Color>(Color(0xffffff6f)),
    ),
    confirmButtonStyle: ButtonStyle(
      foregroundColor: MaterialStatePropertyAll<Color>(Color(0xffffff7f)),
    ),
    locale: Locale('en'),
  );

  Material findDialogMaterial(WidgetTester tester) {
    return tester.widget<Material>(
      find.descendant(of: find.byType(Dialog), matching: find.byType(Material)).first,
    );
  }

  Material findHeaderMaterial(WidgetTester tester, String text) {
    return tester.widget<Material>(
      find.ancestor(of: find.text(text), matching: find.byType(Material)).first,
    );
  }

  BoxDecoration? findTextDecoration(WidgetTester tester, String date) {
    final Container container = tester.widget<Container>(
      find.ancestor(of: find.text(date), matching: find.byType(Container)).first,
    );
    return container.decoration as BoxDecoration?;
  }

  ShapeDecoration? findDayDecoration(WidgetTester tester, String day) {
    return tester
            .widget<Ink>(find.ancestor(of: find.text(day), matching: find.byType(Ink)))
            .decoration
        as ShapeDecoration?;
  }

  ButtonStyle actionButtonStyle(WidgetTester tester, String text) {
    return tester.widget<TextButton>(find.widgetWithText(TextButton, text)).style!;
  }

  const Size wideWindowSize = Size(1920.0, 1080.0);
  const Size narrowWindowSize = Size(1070.0, 1770.0);

  test('DatePickerThemeData copyWith, ==, hashCode basics', () {
    expect(const DatePickerThemeData(), const DatePickerThemeData().copyWith());
    expect(const DatePickerThemeData().hashCode, const DatePickerThemeData().copyWith().hashCode);
  });

  test('DatePickerThemeData lerp special cases', () {
    const DatePickerThemeData data = DatePickerThemeData();
    expect(identical(DatePickerThemeData.lerp(data, data, 0.5), data), true);
  });

  test('DatePickerThemeData defaults', () {
    const DatePickerThemeData theme = DatePickerThemeData();
    expect(theme.backgroundColor, null);
    expect(theme.elevation, null);
    expect(theme.shadowColor, null);
    expect(theme.surfaceTintColor, null);
    expect(theme.shape, null);
    expect(theme.headerBackgroundColor, null);
    expect(theme.headerForegroundColor, null);
    expect(theme.headerHeadlineStyle, null);
    expect(theme.headerHelpStyle, null);
    expect(theme.weekdayStyle, null);
    expect(theme.dayStyle, null);
    expect(theme.dayForegroundColor, null);
    expect(theme.dayBackgroundColor, null);
    expect(theme.dayOverlayColor, null);
    expect(theme.dayShape, null);
    expect(theme.todayForegroundColor, null);
    expect(theme.todayBackgroundColor, null);
    expect(theme.todayBorder, null);
    expect(theme.yearStyle, null);
    expect(theme.yearForegroundColor, null);
    expect(theme.yearBackgroundColor, null);
    expect(theme.yearOverlayColor, null);
    expect(theme.rangePickerBackgroundColor, null);
    expect(theme.rangePickerElevation, null);
    expect(theme.rangePickerShadowColor, null);
    expect(theme.rangePickerSurfaceTintColor, null);
    expect(theme.rangePickerShape, null);
    expect(theme.rangePickerHeaderBackgroundColor, null);
    expect(theme.rangePickerHeaderForegroundColor, null);
    expect(theme.rangePickerHeaderHeadlineStyle, null);
    expect(theme.rangePickerHeaderHelpStyle, null);
    expect(theme.rangeSelectionBackgroundColor, null);
    expect(theme.rangeSelectionOverlayColor, null);
    expect(theme.dividerColor, null);
    expect(theme.inputDecorationTheme, null);
    expect(theme.cancelButtonStyle, null);
    expect(theme.confirmButtonStyle, null);
    expect(theme.locale, null);
  });

  testWidgets('DatePickerTheme.defaults M3 defaults', (WidgetTester tester) async {
    late final DatePickerThemeData m3; // M3 Defaults
    late final ThemeData theme;
    late final ColorScheme colorScheme;
    late final TextTheme textTheme;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: Builder(
          builder: (BuildContext context) {
            m3 = DatePickerTheme.defaults(context);
            theme = Theme.of(context);
            colorScheme = theme.colorScheme;
            textTheme = theme.textTheme;
            return Container();
          },
        ),
      ),
    );

    expect(m3.backgroundColor, colorScheme.surfaceContainerHigh);
    expect(m3.elevation, 6);
    expect(m3.shadowColor, const Color(0x00000000)); // Colors.transparent
    expect(m3.surfaceTintColor, Colors.transparent);
    expect(m3.shape, RoundedRectangleBorder(borderRadius: BorderRadius.circular(28)));
    expect(m3.headerBackgroundColor, const Color(0x00000000)); // Colors.transparent
    expect(m3.headerForegroundColor, colorScheme.onSurfaceVariant);
    expect(m3.headerHeadlineStyle, textTheme.headlineLarge);
    expect(m3.headerHelpStyle, textTheme.labelLarge);
    expect(m3.weekdayStyle, textTheme.bodyLarge?.apply(color: colorScheme.onSurface));
    expect(m3.dayStyle, textTheme.bodyLarge);
    expect(m3.dayForegroundColor?.resolve(<MaterialState>{}), colorScheme.onSurface);
    expect(
      m3.dayForegroundColor?.resolve(<MaterialState>{MaterialState.selected}),
      colorScheme.onPrimary,
    );
    expect(
      m3.dayForegroundColor?.resolve(<MaterialState>{MaterialState.disabled}),
      colorScheme.onSurface.withOpacity(0.38),
    );
    expect(m3.dayBackgroundColor?.resolve(<MaterialState>{}), null);
    expect(
      m3.dayBackgroundColor?.resolve(<MaterialState>{MaterialState.selected}),
      colorScheme.primary,
    );
    expect(m3.dayOverlayColor?.resolve(<MaterialState>{}), null);
    expect(
      m3.dayOverlayColor?.resolve(<MaterialState>{MaterialState.selected, MaterialState.hovered}),
      colorScheme.onPrimary.withOpacity(0.08),
    );
    expect(
      m3.dayOverlayColor?.resolve(<MaterialState>{MaterialState.selected, MaterialState.focused}),
      colorScheme.onPrimary.withOpacity(0.1),
    );
    expect(
      m3.dayOverlayColor?.resolve(<MaterialState>{MaterialState.hovered}),
      colorScheme.onSurfaceVariant.withOpacity(0.08),
    );
    expect(
      m3.dayOverlayColor?.resolve(<MaterialState>{MaterialState.focused}),
      colorScheme.onSurfaceVariant.withOpacity(0.1),
    );
    expect(
      m3.dayOverlayColor?.resolve(<MaterialState>{MaterialState.pressed}),
      colorScheme.onSurfaceVariant.withOpacity(0.1),
    );
    expect(
      m3.dayOverlayColor?.resolve(<MaterialState>{
        MaterialState.selected,
        MaterialState.hovered,
        MaterialState.focused,
      }),
      colorScheme.onPrimary.withOpacity(0.08),
    );
    expect(
      m3.dayOverlayColor?.resolve(<MaterialState>{
        MaterialState.selected,
        MaterialState.hovered,
        MaterialState.pressed,
      }),
      colorScheme.onPrimary.withOpacity(0.1),
    );
    expect(
      m3.dayOverlayColor?.resolve(<MaterialState>{MaterialState.hovered, MaterialState.focused}),
      colorScheme.onSurfaceVariant.withOpacity(0.08),
    );
    expect(
      m3.dayOverlayColor?.resolve(<MaterialState>{MaterialState.hovered, MaterialState.pressed}),
      colorScheme.onSurfaceVariant.withOpacity(0.1),
    );
    expect(m3.dayShape?.resolve(<MaterialState>{}), const CircleBorder());
    expect(m3.todayForegroundColor?.resolve(<MaterialState>{}), colorScheme.primary);
    expect(
      m3.todayForegroundColor?.resolve(<MaterialState>{MaterialState.disabled}),
      colorScheme.primary.withOpacity(0.38),
    );
    expect(m3.todayBorder, BorderSide(color: colorScheme.primary));
    expect(m3.yearStyle, textTheme.bodyLarge);
    expect(m3.yearForegroundColor?.resolve(<MaterialState>{}), colorScheme.onSurfaceVariant);
    expect(
      m3.yearForegroundColor?.resolve(<MaterialState>{MaterialState.selected}),
      colorScheme.onPrimary,
    );
    expect(
      m3.yearForegroundColor?.resolve(<MaterialState>{MaterialState.disabled}),
      colorScheme.onSurfaceVariant.withOpacity(0.38),
    );
    expect(m3.yearBackgroundColor?.resolve(<MaterialState>{}), null);
    expect(
      m3.yearBackgroundColor?.resolve(<MaterialState>{MaterialState.selected}),
      colorScheme.primary,
    );
    expect(m3.yearOverlayColor?.resolve(<MaterialState>{}), null);
    expect(
      m3.yearOverlayColor?.resolve(<MaterialState>{MaterialState.selected, MaterialState.hovered}),
      colorScheme.onPrimary.withOpacity(0.08),
    );
    expect(
      m3.yearOverlayColor?.resolve(<MaterialState>{MaterialState.selected, MaterialState.focused}),
      colorScheme.onPrimary.withOpacity(0.1),
    );
    expect(
      m3.yearOverlayColor?.resolve(<MaterialState>{MaterialState.hovered}),
      colorScheme.onSurfaceVariant.withOpacity(0.08),
    );
    expect(
      m3.yearOverlayColor?.resolve(<MaterialState>{MaterialState.focused}),
      colorScheme.onSurfaceVariant.withOpacity(0.1),
    );
    expect(
      m3.yearOverlayColor?.resolve(<MaterialState>{MaterialState.pressed}),
      colorScheme.onSurfaceVariant.withOpacity(0.1),
    );
    expect(m3.rangePickerElevation, 0);
    expect(m3.rangePickerShape, const RoundedRectangleBorder());
    expect(m3.rangePickerShadowColor, Colors.transparent);
    expect(m3.rangePickerSurfaceTintColor, Colors.transparent);
    expect(m3.rangeSelectionOverlayColor?.resolve(<MaterialState>{}), null);
    expect(m3.rangePickerHeaderBackgroundColor, Colors.transparent);
    expect(m3.rangePickerHeaderForegroundColor, colorScheme.onSurfaceVariant);
    expect(m3.rangePickerHeaderHeadlineStyle, textTheme.titleLarge);
    expect(m3.rangePickerHeaderHelpStyle, textTheme.titleSmall);
    expect(m3.dividerColor, null);
    expect(m3.inputDecorationTheme, null);
    expect(
      m3.cancelButtonStyle.toString(),
      equalsIgnoringHashCodes(TextButton.styleFrom().toString()),
    );
    expect(
      m3.confirmButtonStyle.toString(),
      equalsIgnoringHashCodes(TextButton.styleFrom().toString()),
    );
    expect(m3.locale, null);
  });

  testWidgets('DatePickerTheme.defaults M2 defaults', (WidgetTester tester) async {
    late final DatePickerThemeData m2; // M2 defaults
    late final ThemeData theme;
    late final ColorScheme colorScheme;
    late final TextTheme textTheme;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Builder(
          builder: (BuildContext context) {
            m2 = DatePickerTheme.defaults(context);
            theme = Theme.of(context);
            colorScheme = theme.colorScheme;
            textTheme = theme.textTheme;
            return Container();
          },
        ),
      ),
    );

    expect(m2.elevation, 24);
    expect(
      m2.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4.0))),
    );
    expect(m2.headerBackgroundColor, colorScheme.primary);
    expect(m2.headerForegroundColor, colorScheme.onPrimary);
    expect(m2.headerHeadlineStyle, textTheme.headlineSmall);
    expect(m2.headerHelpStyle, textTheme.labelSmall);
    expect(
      m2.weekdayStyle,
      textTheme.bodySmall?.apply(color: colorScheme.onSurface.withOpacity(0.60)),
    );
    expect(m2.dayStyle, textTheme.bodySmall);
    expect(m2.dayForegroundColor?.resolve(<MaterialState>{}), colorScheme.onSurface);
    expect(
      m2.dayForegroundColor?.resolve(<MaterialState>{MaterialState.selected}),
      colorScheme.onPrimary,
    );
    expect(
      m2.dayForegroundColor?.resolve(<MaterialState>{MaterialState.disabled}),
      colorScheme.onSurface.withOpacity(0.38),
    );
    expect(m2.dayBackgroundColor?.resolve(<MaterialState>{}), null);
    expect(
      m2.dayBackgroundColor?.resolve(<MaterialState>{MaterialState.selected}),
      colorScheme.primary,
    );
    expect(m2.dayOverlayColor?.resolve(<MaterialState>{}), null);
    expect(
      m2.dayOverlayColor?.resolve(<MaterialState>{MaterialState.selected, MaterialState.hovered}),
      colorScheme.onPrimary.withOpacity(0.08),
    );
    expect(
      m2.dayOverlayColor?.resolve(<MaterialState>{MaterialState.selected, MaterialState.focused}),
      colorScheme.onPrimary.withOpacity(0.12),
    );
    expect(
      m2.dayOverlayColor?.resolve(<MaterialState>{MaterialState.selected, MaterialState.pressed}),
      colorScheme.onPrimary.withOpacity(0.38),
    );
    expect(
      m2.dayOverlayColor?.resolve(<MaterialState>{
        MaterialState.selected,
        MaterialState.hovered,
        MaterialState.focused,
      }),
      colorScheme.onPrimary.withOpacity(0.08),
    );
    expect(
      m2.dayOverlayColor?.resolve(<MaterialState>{
        MaterialState.selected,
        MaterialState.hovered,
        MaterialState.pressed,
      }),
      colorScheme.onPrimary.withOpacity(0.38),
    );
    expect(
      m2.dayOverlayColor?.resolve(<MaterialState>{MaterialState.hovered}),
      colorScheme.onSurfaceVariant.withOpacity(0.08),
    );
    expect(
      m2.dayOverlayColor?.resolve(<MaterialState>{MaterialState.focused}),
      colorScheme.onSurfaceVariant.withOpacity(0.12),
    );
    expect(
      m2.dayOverlayColor?.resolve(<MaterialState>{MaterialState.pressed}),
      colorScheme.onSurfaceVariant.withOpacity(0.12),
    );
    expect(m2.dayShape?.resolve(<MaterialState>{}), const CircleBorder());
    expect(m2.todayForegroundColor?.resolve(<MaterialState>{}), colorScheme.primary);
    expect(
      m2.todayForegroundColor?.resolve(<MaterialState>{MaterialState.disabled}),
      colorScheme.onSurface.withOpacity(0.38),
    );
    expect(m2.todayBorder, BorderSide(color: colorScheme.primary));
    expect(m2.yearStyle, textTheme.bodyLarge);
    expect(m2.rangePickerBackgroundColor, colorScheme.surface);
    expect(m2.rangePickerElevation, 0);
    expect(m2.rangePickerShape, const RoundedRectangleBorder());
    expect(m2.rangePickerShadowColor, Colors.transparent);
    expect(m2.rangePickerSurfaceTintColor, Colors.transparent);
    expect(m2.rangeSelectionOverlayColor?.resolve(<MaterialState>{}), null);
    expect(
      m2.rangeSelectionOverlayColor?.resolve(<MaterialState>{
        MaterialState.selected,
        MaterialState.hovered,
      }),
      colorScheme.onPrimary.withOpacity(0.08),
    );
    expect(
      m2.rangeSelectionOverlayColor?.resolve(<MaterialState>{
        MaterialState.selected,
        MaterialState.focused,
      }),
      colorScheme.onPrimary.withOpacity(0.12),
    );
    expect(
      m2.rangeSelectionOverlayColor?.resolve(<MaterialState>{
        MaterialState.selected,
        MaterialState.pressed,
      }),
      colorScheme.onPrimary.withOpacity(0.38),
    );
    expect(
      m2.rangeSelectionOverlayColor?.resolve(<MaterialState>{MaterialState.hovered}),
      colorScheme.onSurfaceVariant.withOpacity(0.08),
    );
    expect(
      m2.rangeSelectionOverlayColor?.resolve(<MaterialState>{MaterialState.focused}),
      colorScheme.onSurfaceVariant.withOpacity(0.12),
    );
    expect(
      m2.rangeSelectionOverlayColor?.resolve(<MaterialState>{MaterialState.pressed}),
      colorScheme.onSurfaceVariant.withOpacity(0.12),
    );
    expect(m2.rangePickerHeaderBackgroundColor, colorScheme.primary);
    expect(m2.rangePickerHeaderForegroundColor, colorScheme.onPrimary);
    expect(m2.rangePickerHeaderHeadlineStyle, textTheme.headlineSmall);
    expect(m2.rangePickerHeaderHelpStyle, textTheme.labelSmall);
    expect(m2.dividerColor, null);
    expect(m2.inputDecorationTheme, null);
    expect(
      m2.cancelButtonStyle.toString(),
      equalsIgnoringHashCodes(TextButton.styleFrom().toString()),
    );
    expect(
      m2.confirmButtonStyle.toString(),
      equalsIgnoringHashCodes(TextButton.styleFrom().toString()),
    );
    expect(m2.locale, null);
  });

  testWidgets('Default DatePickerThemeData debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const DatePickerThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('DatePickerThemeData implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();

    datePickerTheme.debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(
      description,
      equalsIgnoringHashCodes(<String>[
        'backgroundColor: ${const Color(0xfffffff0)}',
        'elevation: 6.0',
        'shadowColor: ${const Color(0xfffffff1)}',
        'surfaceTintColor: ${const Color(0xfffffff2)}',
        'shape: RoundedRectangleBorder(BorderSide(width: 0.0, style: none), BorderRadius.zero)',
        'headerBackgroundColor: ${const Color(0xfffffff3)}',
        'headerForegroundColor: ${const Color(0xfffffff4)}',
        'headerHeadlineStyle: TextStyle(inherit: true, size: 10.0)',
        'headerHelpStyle: TextStyle(inherit: true, size: 11.0)',
        'weekDayStyle: TextStyle(inherit: true, size: 12.0)',
        'dayStyle: TextStyle(inherit: true, size: 13.0)',
        'dayForegroundColor: WidgetStatePropertyAll(${const Color(0xfffffff5)})',
        'dayBackgroundColor: WidgetStatePropertyAll(${const Color(0xfffffff6)})',
        'dayOverlayColor: WidgetStatePropertyAll(${const Color(0xfffffff7)})',
        'dayShape: WidgetStatePropertyAll(RoundedRectangleBorder(BorderSide(width: 0.0, style: none), BorderRadius.zero))',
        'todayForegroundColor: WidgetStatePropertyAll(${const Color(0xfffffff8)})',
        'todayBackgroundColor: WidgetStatePropertyAll(${const Color(0xfffffff9)})',
        'todayBorder: BorderSide(width: 3.0)',
        'yearStyle: TextStyle(inherit: true, size: 13.0)',
        'yearForegroundColor: WidgetStatePropertyAll(${const Color(0xfffffffa)})',
        'yearBackgroundColor: WidgetStatePropertyAll(${const Color(0xfffffffb)})',
        'yearOverlayColor: WidgetStatePropertyAll(${const Color(0xfffffffc)})',
        'rangePickerBackgroundColor: ${const Color(0xfffffffd)}',
        'rangePickerElevation: 7.0',
        'rangePickerShadowColor: ${const Color(0xfffffffe)}',
        'rangePickerSurfaceTintColor: ${const Color(0xffffffff)}',
        'rangePickerShape: RoundedRectangleBorder(BorderSide(width: 0.0, style: none), BorderRadius.zero)',
        'rangePickerHeaderBackgroundColor: ${const Color(0xffffff0f)}',
        'rangePickerHeaderForegroundColor: ${const Color(0xffffff1f)}',
        'rangePickerHeaderHeadlineStyle: TextStyle(inherit: true, size: 14.0)',
        'rangePickerHeaderHelpStyle: TextStyle(inherit: true, size: 15.0)',
        'rangeSelectionBackgroundColor: ${const Color(0xffffff2f)}',
        'rangeSelectionOverlayColor: WidgetStatePropertyAll(${const Color(0xffffff3f)})',
        'dividerColor: ${const Color(0xffffff4f)}',
        'inputDecorationTheme: InputDecorationTheme#00000(fillColor: ${const Color(0xffffff5f)}, border: UnderlineInputBorder())',
        'cancelButtonStyle: ButtonStyle#00000(foregroundColor: WidgetStatePropertyAll(${const Color(0xffffff6f)}))',
        'confirmButtonStyle: ButtonStyle#00000(foregroundColor: WidgetStatePropertyAll(${const Color(0xffffff7f)}))',
        'locale: en',
      ]),
    );
  });

  testWidgets('DatePickerDialog uses ThemeData datePicker theme (calendar mode)', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(datePickerTheme: datePickerTheme, useMaterial3: true),
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: DatePickerDialog(
                initialDate: DateTime(2023, DateTime.january, 25),
                firstDate: DateTime(2022),
                lastDate: DateTime(2024, DateTime.december, 31),
                currentDate: DateTime(2023, DateTime.january, 24),
              ),
            ),
          ),
        ),
      ),
    );

    final Material material = findDialogMaterial(tester);
    expect(material.color, datePickerTheme.backgroundColor);
    expect(material.elevation, datePickerTheme.elevation);
    expect(material.shadowColor, datePickerTheme.shadowColor);
    expect(material.surfaceTintColor, datePickerTheme.surfaceTintColor);
    expect(material.shape, datePickerTheme.shape);

    final Text selectDate = tester.widget<Text>(find.text('Select date'));
    final Material headerMaterial = findHeaderMaterial(tester, 'Select date');
    expect(selectDate.style?.color, datePickerTheme.headerForegroundColor);
    expect(selectDate.style?.fontSize, datePickerTheme.headerHelpStyle?.fontSize);
    expect(headerMaterial.color, datePickerTheme.headerBackgroundColor);

    final Text weekday = tester.widget<Text>(find.text('W'));
    expect(weekday.style?.color, datePickerTheme.weekdayStyle?.color);
    expect(weekday.style?.fontSize, datePickerTheme.weekdayStyle?.fontSize);

    final Text selectedDate = tester.widget<Text>(find.text('Wed, Jan 25'));
    expect(selectedDate.style?.color, datePickerTheme.headerForegroundColor);
    expect(selectedDate.style?.fontSize, datePickerTheme.headerHeadlineStyle?.fontSize);

    final Text day31 = tester.widget<Text>(find.text('31'));
    final ShapeDecoration day31Decoration = findDayDecoration(tester, '31')!;
    expect(day31.style?.color, datePickerTheme.dayForegroundColor?.resolve(<MaterialState>{}));
    expect(day31.style?.fontSize, datePickerTheme.dayStyle?.fontSize);
    expect(day31Decoration.color, datePickerTheme.dayBackgroundColor?.resolve(<MaterialState>{}));
    expect(day31Decoration.shape, datePickerTheme.dayShape?.resolve(<MaterialState>{}));

    final Text day24 = tester.widget<Text>(find.text('24')); // DatePickerDialog.currentDate
    final ShapeDecoration day24Decoration = findDayDecoration(tester, '24')!;
    final OutlinedBorder day24Shape = day24Decoration.shape as OutlinedBorder;
    expect(day24.style?.fontSize, datePickerTheme.dayStyle?.fontSize);
    expect(day24.style?.color, datePickerTheme.todayForegroundColor?.resolve(<MaterialState>{}));
    expect(day24Decoration.color, datePickerTheme.todayBackgroundColor?.resolve(<MaterialState>{}));
    expect(
      day24Decoration.shape,
      datePickerTheme.dayShape
          ?.resolve(<MaterialState>{})!
          .copyWith(
            side: datePickerTheme.todayBorder?.copyWith(
              color: datePickerTheme.todayForegroundColor?.resolve(<MaterialState>{}),
            ),
          ),
    );
    expect(day24Shape.side.width, datePickerTheme.todayBorder?.width);

    // Test the day overlay color.
    final RenderObject inkFeatures = tester.allRenderObjects.firstWhere(
      (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
    );
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.text('25')));
    await tester.pumpAndSettle();
    expect(
      inkFeatures,
      paints..circle(color: datePickerTheme.dayOverlayColor?.resolve(<MaterialState>{})),
    );

    // Show the year selector.

    await tester.tap(find.text('January 2023'));
    await tester.pumpAndSettle();

    final Text year2022 = tester.widget<Text>(find.text('2022'));
    final BoxDecoration year2022Decoration = findTextDecoration(tester, '2022')!;
    expect(year2022.style?.fontSize, datePickerTheme.yearStyle?.fontSize);
    expect(year2022.style?.color, datePickerTheme.yearForegroundColor?.resolve(<MaterialState>{}));
    expect(
      year2022Decoration.color,
      datePickerTheme.yearBackgroundColor?.resolve(<MaterialState>{}),
    );

    final Text year2023 = tester.widget<Text>(find.text('2023')); // DatePickerDialog.currentDate
    final BoxDecoration year2023Decoration = findTextDecoration(tester, '2023')!;
    expect(year2023.style?.fontSize, datePickerTheme.yearStyle?.fontSize);
    expect(year2023.style?.color, datePickerTheme.todayForegroundColor?.resolve(<MaterialState>{}));
    expect(
      year2023Decoration.color,
      datePickerTheme.todayBackgroundColor?.resolve(<MaterialState>{}),
    );
    expect(year2023Decoration.border?.top.width, datePickerTheme.todayBorder?.width);
    expect(year2023Decoration.border?.bottom.width, datePickerTheme.todayBorder?.width);
    expect(
      year2023Decoration.border?.top.color,
      datePickerTheme.todayForegroundColor?.resolve(<MaterialState>{}),
    );
    expect(
      year2023Decoration.border?.bottom.color,
      datePickerTheme.todayForegroundColor?.resolve(<MaterialState>{}),
    );

    // Test the year overlay color.
    await gesture.moveTo(tester.getCenter(find.text('2024')));
    await tester.pumpAndSettle();
    expect(
      inkFeatures,
      paints..rect(color: datePickerTheme.yearOverlayColor?.resolve(<MaterialState>{})),
    );

    final ButtonStyle cancelButtonStyle = actionButtonStyle(tester, 'Cancel');
    expect(
      cancelButtonStyle.toString(),
      equalsIgnoringHashCodes(datePickerTheme.cancelButtonStyle.toString()),
    );

    final ButtonStyle confirmButtonStyle = actionButtonStyle(tester, 'OK');
    expect(
      confirmButtonStyle.toString(),
      equalsIgnoringHashCodes(datePickerTheme.confirmButtonStyle.toString()),
    );
  });

  testWidgets('DatePickerDialog uses ThemeData datePicker theme (input mode)', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(datePickerTheme: datePickerTheme, useMaterial3: true),
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: DatePickerDialog(
                initialEntryMode: DatePickerEntryMode.input,
                initialDate: DateTime(2023, DateTime.january, 25),
                firstDate: DateTime(2022),
                lastDate: DateTime(2024, DateTime.december, 31),
                currentDate: DateTime(2023, DateTime.january, 24),
              ),
            ),
          ),
        ),
      ),
    );

    final Material material = findDialogMaterial(tester);
    expect(material.color, datePickerTheme.backgroundColor);
    expect(material.elevation, datePickerTheme.elevation);
    expect(material.shadowColor, datePickerTheme.shadowColor);
    expect(material.surfaceTintColor, datePickerTheme.surfaceTintColor);
    expect(material.shape, datePickerTheme.shape);

    final Text selectDate = tester.widget<Text>(find.text('Select date'));
    final Material headerMaterial = findHeaderMaterial(tester, 'Select date');
    expect(selectDate.style?.color, datePickerTheme.headerForegroundColor);
    expect(selectDate.style?.fontSize, datePickerTheme.headerHelpStyle?.fontSize);
    expect(headerMaterial.color, datePickerTheme.headerBackgroundColor);

    final InputDecoration inputDecoration =
        tester.widget<TextField>(find.byType(TextField)).decoration!;
    expect(inputDecoration.fillColor, datePickerTheme.inputDecorationTheme?.fillColor);

    final ButtonStyle cancelButtonStyle = actionButtonStyle(tester, 'Cancel');
    expect(
      cancelButtonStyle.toString(),
      equalsIgnoringHashCodes(datePickerTheme.cancelButtonStyle.toString()),
    );

    final ButtonStyle confirmButtonStyle = actionButtonStyle(tester, 'OK');
    expect(
      confirmButtonStyle.toString(),
      equalsIgnoringHashCodes(datePickerTheme.confirmButtonStyle.toString()),
    );
  });

  testWidgets('DateRangePickerDialog uses ThemeData datePicker theme', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(datePickerTheme: datePickerTheme, useMaterial3: true),
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: DateRangePickerDialog(
                firstDate: DateTime(2023),
                lastDate: DateTime(2023, DateTime.january, 31),
                initialDateRange: DateTimeRange(
                  start: DateTime(2023, DateTime.january, 17),
                  end: DateTime(2023, DateTime.january, 20),
                ),
                currentDate: DateTime(2023, DateTime.january, 23),
              ),
            ),
          ),
        ),
      ),
    );

    final Material material = findDialogMaterial(tester);
    expect(material.color, datePickerTheme.backgroundColor);
    expect(
      tester.widget<Scaffold>(find.byType(Scaffold)).backgroundColor,
      datePickerTheme.rangePickerBackgroundColor,
    );
    expect(material.elevation, datePickerTheme.rangePickerElevation);
    expect(material.shadowColor, datePickerTheme.rangePickerShadowColor);
    expect(material.surfaceTintColor, datePickerTheme.rangePickerSurfaceTintColor);
    expect(material.shape, datePickerTheme.rangePickerShape);

    final AppBar appBar = tester.widget<AppBar>(find.byType(AppBar));
    expect(appBar.backgroundColor, datePickerTheme.rangePickerHeaderBackgroundColor);

    final Text selectRange = tester.widget<Text>(find.text('Select range'));
    expect(selectRange.style?.color, datePickerTheme.rangePickerHeaderForegroundColor);
    expect(selectRange.style?.fontSize, datePickerTheme.rangePickerHeaderHelpStyle?.fontSize);

    final Text selectedDate = tester.widget<Text>(find.text('Jan 17'));
    expect(selectedDate.style?.color, datePickerTheme.rangePickerHeaderForegroundColor);
    expect(selectedDate.style?.fontSize, datePickerTheme.rangePickerHeaderHeadlineStyle?.fontSize);

    // Test the day overlay color.
    final RenderObject inkFeatures = tester.allRenderObjects.firstWhere(
      (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
    );
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.text('16')));
    await tester.pumpAndSettle();
    expect(
      inkFeatures,
      paints..circle(color: datePickerTheme.dayOverlayColor?.resolve(<MaterialState>{})),
    );

    // Test the range selection overlay color.
    await gesture.moveTo(tester.getCenter(find.text('18')));
    await tester.pumpAndSettle();
    expect(
      inkFeatures,
      paints..circle(color: datePickerTheme.rangeSelectionOverlayColor?.resolve(<MaterialState>{})),
    );
  });

  testWidgets('Material2 - DateRangePickerDialog uses ThemeData datePicker theme', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(datePickerTheme: datePickerTheme, useMaterial3: false),
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: DateRangePickerDialog(
                firstDate: DateTime(2023),
                lastDate: DateTime(2023, DateTime.january, 31),
                initialDateRange: DateTimeRange(
                  start: DateTime(2023, DateTime.january, 17),
                  end: DateTime(2023, DateTime.january, 20),
                ),
                currentDate: DateTime(2023, DateTime.january, 23),
              ),
            ),
          ),
        ),
      ),
    );

    final Material material = findDialogMaterial(tester);
    expect(material.color, datePickerTheme.backgroundColor);
    expect(
      tester.widget<Scaffold>(find.byType(Scaffold)).backgroundColor,
      datePickerTheme.rangePickerBackgroundColor,
    );
    expect(material.elevation, datePickerTheme.rangePickerElevation);
    expect(material.shadowColor, datePickerTheme.rangePickerShadowColor);
    expect(material.surfaceTintColor, datePickerTheme.rangePickerSurfaceTintColor);
    expect(material.shape, datePickerTheme.rangePickerShape);

    final AppBar appBar = tester.widget<AppBar>(find.byType(AppBar));
    expect(appBar.backgroundColor, datePickerTheme.rangePickerHeaderBackgroundColor);

    final Text selectRange = tester.widget<Text>(find.text('SELECT RANGE'));
    expect(selectRange.style?.color, datePickerTheme.rangePickerHeaderForegroundColor);
    expect(selectRange.style?.fontSize, datePickerTheme.rangePickerHeaderHelpStyle?.fontSize);

    final Text selectedDate = tester.widget<Text>(find.text('Jan 17'));
    expect(selectedDate.style?.color, datePickerTheme.rangePickerHeaderForegroundColor);
    expect(selectedDate.style?.fontSize, datePickerTheme.rangePickerHeaderHeadlineStyle?.fontSize);

    // Test the day overlay color.
    final RenderObject inkFeatures = tester.allRenderObjects.firstWhere(
      (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
    );
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.text('16')));
    await tester.pumpAndSettle();
    expect(
      inkFeatures,
      paints..circle(color: datePickerTheme.dayOverlayColor?.resolve(<MaterialState>{})),
    );

    // Test the range selection overlay color.
    await gesture.moveTo(tester.getCenter(find.text('18')));
    await tester.pumpAndSettle();
    expect(
      inkFeatures,
      paints..circle(color: datePickerTheme.rangeSelectionOverlayColor?.resolve(<MaterialState>{})),
    );
  });

  testWidgets('Dividers use DatePickerThemeData.dividerColor', (WidgetTester tester) async {
    Future<void> showPicker(WidgetTester tester, Size size) async {
      tester.view.physicalSize = size;
      tester.view.devicePixelRatio = 1.0;
      addTearDown(tester.view.reset);
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(datePickerTheme: datePickerTheme, useMaterial3: true),
          home: Directionality(
            textDirection: TextDirection.ltr,
            child: Material(
              child: Center(
                child: DatePickerDialog(
                  initialDate: DateTime(2023, DateTime.january, 25),
                  firstDate: DateTime(2022),
                  lastDate: DateTime(2024, DateTime.december, 31),
                  currentDate: DateTime(2023, DateTime.january, 24),
                ),
              ),
            ),
          ),
        ),
      );
    }

    await showPicker(tester, wideWindowSize);

    // Test vertical divider.
    final VerticalDivider verticalDivider = tester.widget(find.byType(VerticalDivider));
    expect(verticalDivider.color, datePickerTheme.dividerColor);

    // Test portrait layout.
    await showPicker(tester, narrowWindowSize);

    // Test horizontal divider.
    final Divider horizontalDivider = tester.widget(find.byType(Divider));
    expect(horizontalDivider.color, datePickerTheme.dividerColor);
  });

  testWidgets('DatePicker uses ThemeData.inputDecorationTheme properties '
      'which are null in DatePickerThemeData.inputDecorationTheme', (WidgetTester tester) async {
    Widget buildWidget({
      InputDecorationTheme? inputDecorationTheme,
      DatePickerThemeData? datePickerTheme,
    }) {
      return MaterialApp(
        theme: ThemeData(
          useMaterial3: true,
          inputDecorationTheme: inputDecorationTheme,
          datePickerTheme: datePickerTheme,
        ),
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: DatePickerDialog(
                initialEntryMode: DatePickerEntryMode.input,
                initialDate: DateTime(2023, DateTime.january, 25),
                firstDate: DateTime(2022),
                lastDate: DateTime(2024, DateTime.december, 31),
                currentDate: DateTime(2023, DateTime.january, 24),
              ),
            ),
          ),
        ),
      );
    }

    // Test DatePicker with DatePickerThemeData.inputDecorationTheme.
    await tester.pumpWidget(
      buildWidget(
        inputDecorationTheme: const InputDecorationTheme(filled: true),
        datePickerTheme: datePickerTheme,
      ),
    );
    InputDecoration inputDecoration = tester.widget<TextField>(find.byType(TextField)).decoration!;
    expect(inputDecoration.fillColor, datePickerTheme.inputDecorationTheme!.fillColor);
    expect(inputDecoration.border, datePickerTheme.inputDecorationTheme!.border);

    // Test DatePicker with ThemeData.inputDecorationTheme.
    await tester.pumpWidget(
      buildWidget(
        inputDecorationTheme: const InputDecorationTheme(
          filled: true,
          fillColor: Color(0xFF00FF00),
          border: OutlineInputBorder(),
        ),
      ),
    );
    await tester.pumpAndSettle();

    inputDecoration = tester.widget<TextField>(find.byType(TextField)).decoration!;
    expect(inputDecoration.fillColor, const Color(0xFF00FF00));
    expect(inputDecoration.border, const OutlineInputBorder());
  });

  testWidgets('DatePickerDialog resolves DatePickerTheme.dayOverlayColor states', (
    WidgetTester tester,
  ) async {
    final MaterialStateProperty<Color> dayOverlayColor = MaterialStateProperty.resolveWith<Color>((
      Set<MaterialState> states,
    ) {
      if (states.contains(MaterialState.hovered)) {
        return const Color(0xff00ff00);
      }
      if (states.contains(MaterialState.focused)) {
        return const Color(0xffff00ff);
      }
      if (states.contains(MaterialState.pressed)) {
        return const Color(0xffffff00);
      }
      return Colors.transparent;
    });

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(datePickerTheme: DatePickerThemeData(dayOverlayColor: dayOverlayColor)),
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: Focus(
                child: DatePickerDialog(
                  initialDate: DateTime(2023, DateTime.january, 25),
                  firstDate: DateTime(2022),
                  lastDate: DateTime(2024, DateTime.december, 31),
                  currentDate: DateTime(2023, DateTime.january, 24),
                ),
              ),
            ),
          ),
        ),
      ),
    );

    MaterialInkController findDayGridMaterial(WidgetTester tester) {
      // All days are painted on the same Material widget.
      // Use an arbitrary day to find this Material.
      return Material.of(tester.element(find.text('17')));
    }

    // Test the hover overlay color.
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.text('20')));
    await tester.pumpAndSettle();

    expect(
      findDayGridMaterial(tester),
      paints
        ..circle() // Today decoration.
        ..circle() // Selected day decoration.
        ..circle(color: dayOverlayColor.resolve(<MaterialState>{MaterialState.hovered})),
    );

    // Test the pressed overlay color.
    await gesture.down(tester.getCenter(find.text('20')));
    await tester.pumpAndSettle();
    if (kIsWeb) {
      // An extra circle is painted on the web for the hovered state.
      expect(
        findDayGridMaterial(tester),
        paints
          ..circle() // Today decoration.
          ..circle() // Selected day decoration.
          ..circle(color: dayOverlayColor.resolve(<MaterialState>{MaterialState.hovered}))
          ..circle(color: dayOverlayColor.resolve(<MaterialState>{MaterialState.hovered}))
          ..circle(color: dayOverlayColor.resolve(<MaterialState>{MaterialState.pressed})),
      );
    } else {
      expect(
        findDayGridMaterial(tester),
        paints
          ..circle() // Today decoration.
          ..circle() // Selected day decoration.
          ..circle(color: dayOverlayColor.resolve(<MaterialState>{MaterialState.hovered}))
          ..circle(color: dayOverlayColor.resolve(<MaterialState>{MaterialState.pressed})),
      );
    }

    await gesture.removePointer();
    await tester.pumpAndSettle();

    // Focus day selection.
    for (int i = 0; i < 5; i++) {
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pumpAndSettle();
    }

    // Test the focused overlay color.
    expect(
      findDayGridMaterial(tester),
      paints
        ..circle() // Today decoration.
        ..circle() // Selected day decoration.
        ..circle(color: dayOverlayColor.resolve(<MaterialState>{MaterialState.focused})),
    );
  });

  testWidgets('DatePickerDialog resolves DatePickerTheme.yearOverlayColor states', (
    WidgetTester tester,
  ) async {
    final MaterialStateProperty<Color> yearOverlayColor = MaterialStateProperty.resolveWith<Color>((
      Set<MaterialState> states,
    ) {
      if (states.contains(MaterialState.hovered)) {
        return const Color(0xff00ff00);
      }
      if (states.contains(MaterialState.focused)) {
        return const Color(0xffff00ff);
      }
      if (states.contains(MaterialState.pressed)) {
        return const Color(0xffffff00);
      }
      return Colors.transparent;
    });

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          datePickerTheme: DatePickerThemeData(yearOverlayColor: yearOverlayColor),
          useMaterial3: true,
        ),
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: Focus(
                child: DatePickerDialog(
                  initialDate: DateTime(2023, DateTime.january, 25),
                  firstDate: DateTime(2022),
                  lastDate: DateTime(2024, DateTime.december, 31),
                  currentDate: DateTime(2023, DateTime.january, 24),
                  initialCalendarMode: DatePickerMode.year,
                ),
              ),
            ),
          ),
        ),
      ),
    );

    // Test the hover overlay color.
    final RenderObject inkFeatures = tester.allRenderObjects.firstWhere(
      (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
    );
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.text('2022')));
    await tester.pumpAndSettle();
    expect(
      inkFeatures,
      paints..rect(color: yearOverlayColor.resolve(<MaterialState>{MaterialState.hovered})),
    );

    // Test the pressed overlay color.
    await gesture.down(tester.getCenter(find.text('2022')));
    await tester.pumpAndSettle();
    expect(
      inkFeatures,
      paints
        ..rect(color: yearOverlayColor.resolve(<MaterialState>{MaterialState.hovered}))
        ..rect(color: yearOverlayColor.resolve(<MaterialState>{MaterialState.pressed})),
    );

    await gesture.removePointer();
    await tester.pumpAndSettle();

    // Focus year selection.
    for (int i = 0; i < 3; i++) {
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pumpAndSettle();
    }

    // Test the focused overlay color.
    expect(
      inkFeatures,
      paints..rect(color: yearOverlayColor.resolve(<MaterialState>{MaterialState.focused})),
    );
  });

  testWidgets('DateRangePickerDialog resolves DatePickerTheme.rangeSelectionOverlayColor states', (
    WidgetTester tester,
  ) async {
    final MaterialStateProperty<Color> rangeSelectionOverlayColor =
        MaterialStateProperty.resolveWith<Color>((Set<MaterialState> states) {
          if (states.contains(MaterialState.hovered)) {
            return const Color(0xff00ff00);
          }
          if (states.contains(MaterialState.pressed)) {
            return const Color(0xffffff00);
          }
          return Colors.transparent;
        });

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          datePickerTheme: DatePickerThemeData(
            rangeSelectionOverlayColor: rangeSelectionOverlayColor,
          ),
          useMaterial3: true,
        ),
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: DateRangePickerDialog(
                firstDate: DateTime(2023),
                lastDate: DateTime(2023, DateTime.january, 31),
                initialDateRange: DateTimeRange(
                  start: DateTime(2023, DateTime.january, 17),
                  end: DateTime(2023, DateTime.january, 20),
                ),
                currentDate: DateTime(2023, DateTime.january, 23),
              ),
            ),
          ),
        ),
      ),
    );

    // Test the hover overlay color.
    final RenderObject inkFeatures = tester.allRenderObjects.firstWhere(
      (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
    );
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.text('18')));
    await tester.pumpAndSettle();
    expect(
      inkFeatures,
      paints
        ..circle(color: rangeSelectionOverlayColor.resolve(<MaterialState>{MaterialState.hovered})),
    );

    // Test the pressed overlay color.
    await gesture.down(tester.getCenter(find.text('18')));
    await tester.pumpAndSettle();
    if (kIsWeb) {
      // An extra circle is painted on the web for the hovered state.
      expect(
        inkFeatures,
        paints
          ..circle(
            color: rangeSelectionOverlayColor.resolve(<MaterialState>{MaterialState.hovered}),
          )
          ..circle(
            color: rangeSelectionOverlayColor.resolve(<MaterialState>{MaterialState.hovered}),
          )
          ..circle(
            color: rangeSelectionOverlayColor.resolve(<MaterialState>{MaterialState.pressed}),
          ),
      );
    } else {
      expect(
        inkFeatures,
        paints
          ..circle(
            color: rangeSelectionOverlayColor.resolve(<MaterialState>{MaterialState.hovered}),
          )
          ..circle(
            color: rangeSelectionOverlayColor.resolve(<MaterialState>{MaterialState.pressed}),
          ),
      );
    }
  });
}
