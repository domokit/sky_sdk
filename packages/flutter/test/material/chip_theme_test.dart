// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

RenderBox getMaterialBox(WidgetTester tester) {
  return tester.firstRenderObject<RenderBox>(
    find.descendant(of: find.byType(RawChip), matching: find.byType(CustomPaint)),
  );
}

Material getMaterial(WidgetTester tester) {
  return tester.widget<Material>(
    find.descendant(of: find.byType(RawChip), matching: find.byType(Material)),
  );
}

IconThemeData getIconData(WidgetTester tester) {
  final IconTheme iconTheme = tester.firstWidget(
    find.descendant(of: find.byType(RawChip), matching: find.byType(IconTheme)),
  );
  return iconTheme.data;
}

DefaultTextStyle getLabelStyle(WidgetTester tester) {
  return tester.widget(
    find.descendant(of: find.byType(RawChip), matching: find.byType(DefaultTextStyle)).last,
  );
}

TextStyle? getIconStyle(WidgetTester tester, IconData icon) {
  final RichText iconRichText = tester.widget<RichText>(
    find.descendant(of: find.byIcon(icon).first, matching: find.byType(RichText)),
  );
  return iconRichText.text.style;
}

void main() {
  test('ChipThemeData copyWith, ==, hashCode basics', () {
    expect(const ChipThemeData(), const ChipThemeData().copyWith());
    expect(const ChipThemeData().hashCode, const ChipThemeData().copyWith().hashCode);
  });

  test('ChipThemeData lerp special cases', () {
    expect(ChipThemeData.lerp(null, null, 0), null);
    const ChipThemeData data = ChipThemeData();
    expect(identical(ChipThemeData.lerp(data, data, 0.5), data), true);
  });

  test('ChipThemeData defaults', () {
    const ChipThemeData themeData = ChipThemeData();
    expect(themeData.color, null);
    expect(themeData.backgroundColor, null);
    expect(themeData.deleteIconColor, null);
    expect(themeData.disabledColor, null);
    expect(themeData.selectedColor, null);
    expect(themeData.secondarySelectedColor, null);
    expect(themeData.shadowColor, null);
    expect(themeData.surfaceTintColor, null);
    expect(themeData.selectedShadowColor, null);
    expect(themeData.showCheckmark, null);
    expect(themeData.checkmarkColor, null);
    expect(themeData.labelPadding, null);
    expect(themeData.padding, null);
    expect(themeData.side, null);
    expect(themeData.shape, null);
    expect(themeData.labelStyle, null);
    expect(themeData.secondaryLabelStyle, null);
    expect(themeData.brightness, null);
    expect(themeData.elevation, null);
    expect(themeData.pressElevation, null);
    expect(themeData.iconTheme, null);
    expect(themeData.avatarBoxConstraints, null);
    expect(themeData.deleteIconBoxConstraints, null);
  });

  testWidgets('Default ChipThemeData debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const ChipThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('ChipThemeData implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const ChipThemeData(
      color: MaterialStatePropertyAll<Color>(Color(0xfffffff0)),
      backgroundColor: Color(0xfffffff1),
      deleteIconColor: Color(0xfffffff2),
      disabledColor: Color(0xfffffff3),
      selectedColor: Color(0xfffffff4),
      secondarySelectedColor: Color(0xfffffff5),
      shadowColor: Color(0xfffffff6),
      surfaceTintColor: Color(0xfffffff7),
      selectedShadowColor: Color(0xfffffff8),
      showCheckmark: true,
      checkmarkColor: Color(0xfffffff9),
      labelPadding: EdgeInsets.all(1),
      padding: EdgeInsets.all(2),
      side: BorderSide(width: 10),
      shape: RoundedRectangleBorder(),
      labelStyle: TextStyle(fontSize: 10),
      secondaryLabelStyle: TextStyle(fontSize: 20),
      brightness: Brightness.dark,
      elevation: 5,
      pressElevation: 6,
      iconTheme: IconThemeData(color: Color(0xffffff10)),
      avatarBoxConstraints: BoxConstraints.tightForFinite(),
      deleteIconBoxConstraints: BoxConstraints.tightForFinite(),
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(
      description,
      equalsIgnoringHashCodes(<String>[
        'color: WidgetStatePropertyAll(${const Color(0xfffffff0)})',
        'backgroundColor: ${const Color(0xfffffff1)}',
        'deleteIconColor: ${const Color(0xfffffff2)}',
        'disabledColor: ${const Color(0xfffffff3)}',
        'selectedColor: ${const Color(0xfffffff4)}',
        'secondarySelectedColor: ${const Color(0xfffffff5)}',
        'shadowColor: ${const Color(0xfffffff6)}',
        'surfaceTintColor: ${const Color(0xfffffff7)}',
        'selectedShadowColor: ${const Color(0xfffffff8)}',
        'showCheckmark: true',
        'checkMarkColor: ${const Color(0xfffffff9)}',
        'labelPadding: EdgeInsets.all(1.0)',
        'padding: EdgeInsets.all(2.0)',
        'side: BorderSide(width: 10.0)',
        'shape: RoundedRectangleBorder(BorderSide(width: 0.0, style: none), BorderRadius.zero)',
        'labelStyle: TextStyle(inherit: true, size: 10.0)',
        'secondaryLabelStyle: TextStyle(inherit: true, size: 20.0)',
        'brightness: dark',
        'elevation: 5.0',
        'pressElevation: 6.0',
        'iconTheme: IconThemeData#00000(color: ${const Color(0xffffff10)})',
        'avatarBoxConstraints: BoxConstraints(unconstrained)',
        'deleteIconBoxConstraints: BoxConstraints(unconstrained)',
      ]),
    );
  });

  testWidgets('Material3 - Chip uses ThemeData chip theme', (WidgetTester tester) async {
    const ChipThemeData chipTheme = ChipThemeData(
      backgroundColor: Color(0xff112233),
      elevation: 4,
      padding: EdgeInsets.all(50),
      labelPadding: EdgeInsets.all(25),
      shape: RoundedRectangleBorder(),
      labelStyle: TextStyle(fontSize: 32),
      iconTheme: IconThemeData(color: Color(0xff332211)),
    );
    final ThemeData theme = ThemeData(chipTheme: chipTheme);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(chipTheme: chipTheme),
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: RawChip(
                avatar: const Icon(Icons.add),
                label: const SizedBox(width: 100, height: 100),
                onSelected: (bool newValue) {},
              ),
            ),
          ),
        ),
      ),
    );

    final RenderBox materialBox = getMaterialBox(tester);
    expect(materialBox, paints..rect(color: chipTheme.backgroundColor));
    expect(getMaterial(tester).elevation, chipTheme.elevation);
    expect(
      tester.getSize(find.byType(RawChip)),
      const Size(402, 252),
    ); // label + padding + labelPadding
    expect(
      getMaterial(tester).shape,
      chipTheme.shape?.copyWith(side: BorderSide(color: theme.colorScheme.outlineVariant)),
    );
    expect(getLabelStyle(tester).style.fontSize, 32);
    expect(getIconData(tester).color, chipTheme.iconTheme!.color);
  });

  testWidgets('Material2 - Chip uses ThemeData chip theme', (WidgetTester tester) async {
    const ChipThemeData chipTheme = ChipThemeData(
      backgroundColor: Color(0xff112233),
      elevation: 4,
      padding: EdgeInsets.all(50),
      labelPadding: EdgeInsets.all(25),
      shape: RoundedRectangleBorder(),
      labelStyle: TextStyle(fontSize: 32),
      iconTheme: IconThemeData(color: Color(0xff332211)),
    );

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(chipTheme: chipTheme, useMaterial3: false),
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: RawChip(
                avatar: const Icon(Icons.add),
                label: const SizedBox(width: 100, height: 100),
                onSelected: (bool newValue) {},
              ),
            ),
          ),
        ),
      ),
    );

    final RenderBox materialBox = getMaterialBox(tester);
    expect(materialBox, paints..rect(color: chipTheme.backgroundColor));
    expect(getMaterial(tester).elevation, chipTheme.elevation);
    expect(
      tester.getSize(find.byType(RawChip)),
      const Size(400, 250),
    ); // label + padding + labelPadding
    expect(getMaterial(tester).shape, chipTheme.shape);
    expect(getLabelStyle(tester).style.fontSize, 32);
    expect(getIconData(tester).color, chipTheme.iconTheme!.color);
  });

  testWidgets('Material3 - Chip uses local ChipTheme', (WidgetTester tester) async {
    const ChipThemeData chipTheme = ChipThemeData(
      backgroundColor: Color(0xff112233),
      elevation: 4,
      padding: EdgeInsets.all(50),
      labelPadding: EdgeInsets.all(25),
      labelStyle: TextStyle(fontSize: 32),
      shape: RoundedRectangleBorder(),
      iconTheme: IconThemeData(color: Color(0xff332211)),
    );
    final ThemeData theme = ThemeData(chipTheme: const ChipThemeData());

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: ChipTheme(
          data: chipTheme,
          child: Builder(
            builder: (BuildContext context) {
              return Directionality(
                textDirection: TextDirection.ltr,
                child: Material(
                  child: Center(
                    child: RawChip(
                      avatar: const Icon(Icons.add),
                      label: const SizedBox(width: 100, height: 100),
                      onSelected: (bool newValue) {},
                    ),
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    final RenderBox materialBox = getMaterialBox(tester);
    expect(materialBox, paints..rect(color: chipTheme.backgroundColor));
    expect(
      tester.getSize(find.byType(RawChip)),
      const Size(402, 252),
    ); // label + padding + labelPadding
    expect(getMaterial(tester).elevation, chipTheme.elevation);
    expect(
      getMaterial(tester).shape,
      chipTheme.shape?.copyWith(side: BorderSide(color: theme.colorScheme.outlineVariant)),
    );
    expect(getLabelStyle(tester).style.fontSize, 32);
    expect(getIconData(tester).color, chipTheme.iconTheme!.color);
  });

  testWidgets('Material2 - Chip uses local ChipTheme', (WidgetTester tester) async {
    const ChipThemeData chipTheme = ChipThemeData(
      backgroundColor: Color(0xff112233),
      elevation: 4,
      padding: EdgeInsets.all(50),
      labelPadding: EdgeInsets.all(25),
      labelStyle: TextStyle(fontSize: 32),
      shape: RoundedRectangleBorder(),
      iconTheme: IconThemeData(color: Color(0xff332211)),
    );

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(chipTheme: const ChipThemeData(), useMaterial3: false),
        home: ChipTheme(
          data: chipTheme,
          child: Builder(
            builder: (BuildContext context) {
              return Directionality(
                textDirection: TextDirection.ltr,
                child: Material(
                  child: Center(
                    child: RawChip(
                      avatar: const Icon(Icons.add),
                      label: const SizedBox(width: 100, height: 100),
                      onSelected: (bool newValue) {},
                    ),
                  ),
                ),
              );
            },
          ),
        ),
      ),
    );

    final RenderBox materialBox = getMaterialBox(tester);
    expect(materialBox, paints..rect(color: chipTheme.backgroundColor));
    expect(
      tester.getSize(find.byType(RawChip)),
      const Size(400, 250),
    ); // label + padding + labelPadding
    expect(getMaterial(tester).elevation, chipTheme.elevation);
    expect(getMaterial(tester).shape, chipTheme.shape);
    expect(getLabelStyle(tester).style.fontSize, 32);
    expect(getIconData(tester).color, chipTheme.iconTheme!.color);
  });

  testWidgets('Chip properties overrides ChipTheme', (WidgetTester tester) async {
    const ChipThemeData chipTheme = ChipThemeData(
      backgroundColor: Color(0xff112233),
      elevation: 4,
      padding: EdgeInsets.all(50),
      labelPadding: EdgeInsets.all(25),
      labelStyle: TextStyle(fontSize: 32),
      shape: RoundedRectangleBorder(),
      iconTheme: IconThemeData(color: Color(0xff332211)),
    );

    const Color backgroundColor = Color(0xff000000);
    const double elevation = 6.0;
    const EdgeInsets padding = EdgeInsets.all(10);
    const EdgeInsets labelPadding = EdgeInsets.all(5);
    const TextStyle labelStyle = TextStyle(fontSize: 20);
    const RoundedRectangleBorder shape = RoundedRectangleBorder(
      side: BorderSide(color: Color(0xff0000ff)),
    );
    const IconThemeData iconTheme = IconThemeData(color: Color(0xff00ff00));

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(chipTheme: chipTheme),
        home: Builder(
          builder: (BuildContext context) {
            return Directionality(
              textDirection: TextDirection.ltr,
              child: Material(
                child: Center(
                  child: RawChip(
                    backgroundColor: backgroundColor,
                    elevation: elevation,
                    padding: padding,
                    labelPadding: labelPadding,
                    labelStyle: labelStyle,
                    shape: shape,
                    iconTheme: iconTheme,
                    avatar: const Icon(Icons.add),
                    label: const SizedBox(width: 100, height: 100),
                    onSelected: (bool newValue) {},
                  ),
                ),
              ),
            );
          },
        ),
      ),
    );

    final RenderBox materialBox = getMaterialBox(tester);
    expect(materialBox, paints..rect(color: backgroundColor));
    expect(
      tester.getSize(find.byType(RawChip)),
      const Size(242, 132),
    ); // label + padding + labelPadding
    expect(getMaterial(tester).elevation, elevation);
    expect(getMaterial(tester).shape, shape);
    expect(getLabelStyle(tester).style.fontSize, labelStyle.fontSize);
    expect(getIconData(tester).color, iconTheme.color);
  });

  testWidgets('Material3 - Chip uses constructor parameters', (WidgetTester tester) async {
    const Color backgroundColor = Color(0xff332211);
    const double elevation = 3;
    const double fontSize = 32;
    const OutlinedBorder shape = CircleBorder(side: BorderSide(color: Color(0xff0000ff)));
    const IconThemeData iconTheme = IconThemeData(color: Color(0xff443322));

    await tester.pumpWidget(
      MaterialApp(
        home: Builder(
          builder: (BuildContext context) {
            return Directionality(
              textDirection: TextDirection.ltr,
              child: Material(
                child: Center(
                  child: RawChip(
                    backgroundColor: backgroundColor,
                    elevation: elevation,
                    padding: const EdgeInsets.all(50),
                    labelPadding: const EdgeInsets.all(25),
                    labelStyle: const TextStyle(fontSize: fontSize),
                    shape: shape,
                    iconTheme: iconTheme,
                    avatar: const Icon(Icons.add),
                    label: const SizedBox(width: 100, height: 100),
                    onSelected: (bool newValue) {},
                  ),
                ),
              ),
            );
          },
        ),
      ),
    );

    final RenderBox materialBox = getMaterialBox(tester);
    expect(materialBox, paints..circle(color: backgroundColor));
    expect(
      tester.getSize(find.byType(RawChip)),
      const Size(402, 252),
    ); // label + padding + labelPadding
    expect(getMaterial(tester).elevation, elevation);
    expect(getMaterial(tester).shape, shape);
    expect(getLabelStyle(tester).style.fontSize, 32);
    expect(getIconData(tester).color, iconTheme.color);
  });

  testWidgets('Material2 - Chip uses constructor parameters', (WidgetTester tester) async {
    const Color backgroundColor = Color(0xff332211);
    const double elevation = 3;
    const double fontSize = 32;
    const OutlinedBorder shape = CircleBorder();
    const IconThemeData iconTheme = IconThemeData(color: Color(0xff443322));

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Builder(
          builder: (BuildContext context) {
            return Directionality(
              textDirection: TextDirection.ltr,
              child: Material(
                child: Center(
                  child: RawChip(
                    backgroundColor: backgroundColor,
                    elevation: elevation,
                    padding: const EdgeInsets.all(50),
                    labelPadding: const EdgeInsets.all(25),
                    labelStyle: const TextStyle(fontSize: fontSize),
                    shape: shape,
                    iconTheme: iconTheme,
                    avatar: const Icon(Icons.add),
                    label: const SizedBox(width: 100, height: 100),
                    onSelected: (bool newValue) {},
                  ),
                ),
              ),
            );
          },
        ),
      ),
    );

    final RenderBox materialBox = getMaterialBox(tester);
    expect(materialBox, paints..circle(color: backgroundColor));
    expect(
      tester.getSize(find.byType(RawChip)),
      const Size(400, 250),
    ); // label + padding + labelPadding
    expect(getMaterial(tester).elevation, elevation);
    expect(getMaterial(tester).shape, shape);
    expect(getLabelStyle(tester).style.fontSize, 32);
    expect(getIconData(tester).color, iconTheme.color);
  });

  testWidgets('ChipTheme.fromDefaults', (WidgetTester tester) async {
    const TextStyle labelStyle = TextStyle();
    ChipThemeData chipTheme = ChipThemeData.fromDefaults(
      brightness: Brightness.light,
      secondaryColor: Colors.red,
      labelStyle: labelStyle,
    );
    expect(chipTheme.backgroundColor, Colors.black.withAlpha(0x1f));
    expect(chipTheme.deleteIconColor, Colors.black.withAlpha(0xde));
    expect(chipTheme.disabledColor, Colors.black.withAlpha(0x0c));
    expect(chipTheme.selectedColor, Colors.black.withAlpha(0x3d));
    expect(chipTheme.secondarySelectedColor, Colors.red.withAlpha(0x3d));
    expect(chipTheme.shadowColor, Colors.black);
    expect(chipTheme.surfaceTintColor, null);
    expect(chipTheme.selectedShadowColor, Colors.black);
    expect(chipTheme.showCheckmark, true);
    expect(chipTheme.checkmarkColor, null);
    expect(chipTheme.labelPadding, null);
    expect(chipTheme.padding, const EdgeInsets.all(4.0));
    expect(chipTheme.side, null);
    expect(chipTheme.shape, null);
    expect(chipTheme.labelStyle, labelStyle.copyWith(color: Colors.black.withAlpha(0xde)));
    expect(chipTheme.secondaryLabelStyle, labelStyle.copyWith(color: Colors.red.withAlpha(0xde)));
    expect(chipTheme.brightness, Brightness.light);
    expect(chipTheme.elevation, 0.0);
    expect(chipTheme.pressElevation, 8.0);

    chipTheme = ChipThemeData.fromDefaults(
      brightness: Brightness.dark,
      secondaryColor: Colors.tealAccent[200]!,
      labelStyle: const TextStyle(),
    );
    expect(chipTheme.backgroundColor, Colors.white.withAlpha(0x1f));
    expect(chipTheme.deleteIconColor, Colors.white.withAlpha(0xde));
    expect(chipTheme.disabledColor, Colors.white.withAlpha(0x0c));
    expect(chipTheme.selectedColor, Colors.white.withAlpha(0x3d));
    expect(chipTheme.secondarySelectedColor, Colors.tealAccent[200]!.withAlpha(0x3d));
    expect(chipTheme.shadowColor, Colors.black);
    expect(chipTheme.selectedShadowColor, Colors.black);
    expect(chipTheme.showCheckmark, true);
    expect(chipTheme.checkmarkColor, null);
    expect(chipTheme.labelPadding, null);
    expect(chipTheme.padding, const EdgeInsets.all(4.0));
    expect(chipTheme.side, null);
    expect(chipTheme.shape, null);
    expect(chipTheme.labelStyle, labelStyle.copyWith(color: Colors.white.withAlpha(0xde)));
    expect(
      chipTheme.secondaryLabelStyle,
      labelStyle.copyWith(color: Colors.tealAccent[200]!.withAlpha(0xde)),
    );
    expect(chipTheme.brightness, Brightness.dark);
    expect(chipTheme.elevation, 0.0);
    expect(chipTheme.pressElevation, 8.0);
  });

  testWidgets('ChipThemeData generates correct opacities for defaults', (
    WidgetTester tester,
  ) async {
    const Color customColor1 = Color(0xcafefeed);
    const Color customColor2 = Color(0xdeadbeef);
    final TextStyle customStyle = ThemeData.fallback().textTheme.bodyLarge!.copyWith(
      color: customColor2,
    );

    final ChipThemeData lightTheme = ChipThemeData.fromDefaults(
      secondaryColor: customColor1,
      brightness: Brightness.light,
      labelStyle: customStyle,
    );

    expect(lightTheme.backgroundColor, equals(Colors.black.withAlpha(0x1f)));
    expect(lightTheme.deleteIconColor, equals(Colors.black.withAlpha(0xde)));
    expect(lightTheme.disabledColor, equals(Colors.black.withAlpha(0x0c)));
    expect(lightTheme.selectedColor, equals(Colors.black.withAlpha(0x3d)));
    expect(lightTheme.secondarySelectedColor, equals(customColor1.withAlpha(0x3d)));
    expect(lightTheme.labelPadding, isNull);
    expect(lightTheme.padding, equals(const EdgeInsets.all(4.0)));
    expect(lightTheme.side, isNull);
    expect(lightTheme.shape, isNull);
    expect(lightTheme.labelStyle?.color, equals(Colors.black.withAlpha(0xde)));
    expect(lightTheme.secondaryLabelStyle?.color, equals(customColor1.withAlpha(0xde)));
    expect(lightTheme.brightness, equals(Brightness.light));

    final ChipThemeData darkTheme = ChipThemeData.fromDefaults(
      secondaryColor: customColor1,
      brightness: Brightness.dark,
      labelStyle: customStyle,
    );

    expect(darkTheme.backgroundColor, equals(Colors.white.withAlpha(0x1f)));
    expect(darkTheme.deleteIconColor, equals(Colors.white.withAlpha(0xde)));
    expect(darkTheme.disabledColor, equals(Colors.white.withAlpha(0x0c)));
    expect(darkTheme.selectedColor, equals(Colors.white.withAlpha(0x3d)));
    expect(darkTheme.secondarySelectedColor, equals(customColor1.withAlpha(0x3d)));
    expect(darkTheme.labelPadding, isNull);
    expect(darkTheme.padding, equals(const EdgeInsets.all(4.0)));
    expect(darkTheme.side, isNull);
    expect(darkTheme.shape, isNull);
    expect(darkTheme.labelStyle?.color, equals(Colors.white.withAlpha(0xde)));
    expect(darkTheme.secondaryLabelStyle?.color, equals(customColor1.withAlpha(0xde)));
    expect(darkTheme.brightness, equals(Brightness.dark));

    final ChipThemeData customTheme = ChipThemeData.fromDefaults(
      primaryColor: customColor1,
      secondaryColor: customColor2,
      labelStyle: customStyle,
    );

    //expect(customTheme.backgroundColor, equals(customColor1.withAlpha(0x1f)));
    expect(customTheme.deleteIconColor, equals(customColor1.withAlpha(0xde)));
    expect(customTheme.disabledColor, equals(customColor1.withAlpha(0x0c)));
    expect(customTheme.selectedColor, equals(customColor1.withAlpha(0x3d)));
    expect(customTheme.secondarySelectedColor, equals(customColor2.withAlpha(0x3d)));
    expect(customTheme.labelPadding, isNull);
    expect(customTheme.padding, equals(const EdgeInsets.all(4.0)));
    expect(customTheme.side, isNull);
    expect(customTheme.shape, isNull);
    expect(customTheme.labelStyle?.color, equals(customColor1.withAlpha(0xde)));
    expect(customTheme.secondaryLabelStyle?.color, equals(customColor2.withAlpha(0xde)));
    expect(customTheme.brightness, equals(Brightness.light));
  });

  testWidgets('ChipThemeData lerps correctly', (WidgetTester tester) async {
    final ChipThemeData chipThemeBlack = ChipThemeData.fromDefaults(
      secondaryColor: Colors.black,
      brightness: Brightness.dark,
      labelStyle: ThemeData.fallback().textTheme.bodyLarge!.copyWith(color: Colors.black),
    ).copyWith(
      elevation: 1.0,
      labelPadding: const EdgeInsets.symmetric(horizontal: 8.0),
      shape: const StadiumBorder(),
      side: const BorderSide(),
      pressElevation: 4.0,
      shadowColor: Colors.black,
      surfaceTintColor: Colors.black,
      selectedShadowColor: Colors.black,
      showCheckmark: false,
      checkmarkColor: Colors.black,
      iconTheme: const IconThemeData(size: 26.0),
    );
    final ChipThemeData chipThemeWhite = ChipThemeData.fromDefaults(
      secondaryColor: Colors.white,
      brightness: Brightness.light,
      labelStyle: ThemeData.fallback().textTheme.bodyLarge!.copyWith(color: Colors.white),
    ).copyWith(
      padding: const EdgeInsets.all(2.0),
      labelPadding: const EdgeInsets.only(top: 8.0, bottom: 8.0),
      shape: const BeveledRectangleBorder(),
      side: const BorderSide(color: Colors.white),
      elevation: 5.0,
      pressElevation: 10.0,
      shadowColor: Colors.white,
      surfaceTintColor: Colors.white,
      selectedShadowColor: Colors.white,
      showCheckmark: true,
      checkmarkColor: Colors.white,
      iconTheme: const IconThemeData(size: 22.0),
    );

    final ChipThemeData lerp = ChipThemeData.lerp(chipThemeBlack, chipThemeWhite, 0.5)!;
    const Color middleGrey = Color(0xff7f7f7f);
    expect(lerp.backgroundColor, isSameColorAs(middleGrey.withAlpha(0x1f)));
    expect(lerp.deleteIconColor, isSameColorAs(middleGrey.withAlpha(0xde)));
    expect(lerp.disabledColor, isSameColorAs(middleGrey.withAlpha(0x0c)));
    expect(lerp.selectedColor, isSameColorAs(middleGrey.withAlpha(0x3d)));
    expect(lerp.secondarySelectedColor, isSameColorAs(middleGrey.withAlpha(0x3d)));
    expect(lerp.shadowColor, isSameColorAs(middleGrey));
    expect(lerp.surfaceTintColor, isSameColorAs(middleGrey));
    expect(lerp.selectedShadowColor, isSameColorAs(middleGrey));
    expect(lerp.showCheckmark, equals(true));
    expect(lerp.labelPadding, equals(const EdgeInsets.all(4.0)));
    expect(lerp.padding, equals(const EdgeInsets.all(3.0)));
    expect(lerp.side!.color, isSameColorAs(middleGrey));
    expect(lerp.shape, isA<BeveledRectangleBorder>());
    expect(lerp.labelStyle?.color, isSameColorAs(middleGrey.withAlpha(0xde)));
    expect(lerp.secondaryLabelStyle?.color, isSameColorAs(middleGrey.withAlpha(0xde)));
    expect(lerp.brightness, equals(Brightness.light));
    expect(lerp.elevation, 3.0);
    expect(lerp.pressElevation, 7.0);
    expect(lerp.checkmarkColor, isSameColorAs(middleGrey));
    expect(lerp.iconTheme, const IconThemeData(size: 24.0));

    expect(ChipThemeData.lerp(null, null, 0.25), isNull);

    final ChipThemeData lerpANull25 = ChipThemeData.lerp(null, chipThemeWhite, 0.25)!;
    expect(lerpANull25.backgroundColor, isSameColorAs(Colors.black.withAlpha(0x08)));
    expect(lerpANull25.deleteIconColor, isSameColorAs(Colors.black.withAlpha(0x38)));
    expect(lerpANull25.disabledColor, isSameColorAs(Colors.black.withAlpha(0x03)));
    expect(lerpANull25.selectedColor, isSameColorAs(Colors.black.withAlpha(0x0f)));
    expect(lerpANull25.secondarySelectedColor, isSameColorAs(Colors.white.withAlpha(0x0f)));
    expect(lerpANull25.shadowColor, isSameColorAs(Colors.white.withAlpha(0x40)));
    expect(lerpANull25.surfaceTintColor, isSameColorAs(Colors.white.withAlpha(0x40)));
    expect(lerpANull25.selectedShadowColor, isSameColorAs(Colors.white.withAlpha(0x40)));
    expect(lerpANull25.showCheckmark, equals(true));
    expect(lerpANull25.labelPadding, equals(const EdgeInsets.only(top: 2.0, bottom: 2.0)));
    expect(lerpANull25.padding, equals(const EdgeInsets.all(0.5)));
    expect(lerpANull25.side!.color, isSameColorAs(Colors.white.withAlpha(0x3f)));
    expect(lerpANull25.shape, isA<BeveledRectangleBorder>());
    expect(lerpANull25.labelStyle?.color, isSameColorAs(Colors.black.withAlpha(0x38)));
    expect(lerpANull25.secondaryLabelStyle?.color, isSameColorAs(Colors.white.withAlpha(0x38)));
    expect(lerpANull25.brightness, equals(Brightness.light));
    expect(lerpANull25.elevation, 1.25);
    expect(lerpANull25.pressElevation, 2.5);
    expect(lerpANull25.checkmarkColor, isSameColorAs(Colors.white.withAlpha(0x40)));
    expect(lerpANull25.iconTheme, const IconThemeData(size: 5.5));

    final ChipThemeData lerpANull75 = ChipThemeData.lerp(null, chipThemeWhite, 0.75)!;
    expect(lerpANull75.backgroundColor, isSameColorAs(Colors.black.withAlpha(0x17)));
    expect(lerpANull75.deleteIconColor, isSameColorAs(Colors.black.withAlpha(0xa7)));
    expect(lerpANull75.disabledColor, isSameColorAs(Colors.black.withAlpha(0x09)));
    expect(lerpANull75.selectedColor, isSameColorAs(Colors.black.withAlpha(0x2e)));
    expect(lerpANull75.secondarySelectedColor, isSameColorAs(Colors.white.withAlpha(0x2e)));
    expect(lerpANull75.shadowColor, isSameColorAs(Colors.white.withAlpha(0xbf)));
    expect(lerpANull75.surfaceTintColor, isSameColorAs(Colors.white.withAlpha(0xbf)));
    expect(lerpANull75.selectedShadowColor, isSameColorAs(Colors.white.withAlpha(0xbf)));
    expect(lerpANull75.showCheckmark, equals(true));
    expect(lerpANull75.labelPadding, equals(const EdgeInsets.only(top: 6.0, bottom: 6.0)));
    expect(lerpANull75.padding, equals(const EdgeInsets.all(1.5)));
    expect(lerpANull75.side!.color, isSameColorAs(Colors.white.withAlpha(0xbf)));
    expect(lerpANull75.shape, isA<BeveledRectangleBorder>());
    expect(lerpANull75.labelStyle?.color, isSameColorAs(Colors.black.withAlpha(0xa7)));
    expect(lerpANull75.secondaryLabelStyle?.color, isSameColorAs(Colors.white.withAlpha(0xa7)));
    expect(lerpANull75.brightness, equals(Brightness.light));
    expect(lerpANull75.elevation, 3.75);
    expect(lerpANull75.pressElevation, 7.5);
    expect(lerpANull75.checkmarkColor, isSameColorAs(Colors.white.withAlpha(0xbf)));
    expect(lerpANull75.iconTheme, const IconThemeData(size: 16.5));

    final ChipThemeData lerpBNull25 = ChipThemeData.lerp(chipThemeBlack, null, 0.25)!;
    expect(lerpBNull25.backgroundColor, isSameColorAs(Colors.white.withAlpha(0x17)));
    expect(lerpBNull25.deleteIconColor, isSameColorAs(Colors.white.withAlpha(0xa7)));
    expect(lerpBNull25.disabledColor, isSameColorAs(Colors.white.withAlpha(0x09)));
    expect(lerpBNull25.selectedColor, isSameColorAs(Colors.white.withAlpha(0x2e)));
    expect(lerpBNull25.secondarySelectedColor, isSameColorAs(Colors.black.withAlpha(0x2e)));
    expect(lerpBNull25.shadowColor, isSameColorAs(Colors.black.withAlpha(0xbf)));
    expect(lerpBNull25.surfaceTintColor, isSameColorAs(Colors.black.withAlpha(0xbf)));
    expect(lerpBNull25.selectedShadowColor, isSameColorAs(Colors.black.withAlpha(0xbf)));
    expect(lerpBNull25.showCheckmark, equals(false));
    expect(lerpBNull25.labelPadding, equals(const EdgeInsets.only(left: 6.0, right: 6.0)));
    expect(lerpBNull25.padding, equals(const EdgeInsets.all(3.0)));
    expect(lerpBNull25.side!.color, isSameColorAs(Colors.black.withAlpha(0x3f)));
    expect(lerpBNull25.shape, isA<StadiumBorder>());
    expect(lerpBNull25.labelStyle?.color, isSameColorAs(Colors.white.withAlpha(0xa7)));
    expect(lerpBNull25.secondaryLabelStyle?.color, isSameColorAs(Colors.black.withAlpha(0xa7)));
    expect(lerpBNull25.brightness, equals(Brightness.dark));
    expect(lerpBNull25.elevation, 0.75);
    expect(lerpBNull25.pressElevation, 3.0);
    expect(lerpBNull25.checkmarkColor, isSameColorAs(Colors.black.withAlpha(0xbf)));
    expect(lerpBNull25.iconTheme, const IconThemeData(size: 19.5));

    final ChipThemeData lerpBNull75 = ChipThemeData.lerp(chipThemeBlack, null, 0.75)!;
    expect(lerpBNull75.backgroundColor, isSameColorAs(Colors.white.withAlpha(0x08)));
    expect(lerpBNull75.deleteIconColor, isSameColorAs(Colors.white.withAlpha(0x38)));
    expect(lerpBNull75.disabledColor, isSameColorAs(Colors.white.withAlpha(0x03)));
    expect(lerpBNull75.selectedColor, isSameColorAs(Colors.white.withAlpha(0x0f)));
    expect(lerpBNull75.secondarySelectedColor, isSameColorAs(Colors.black.withAlpha(0x0f)));
    expect(lerpBNull75.shadowColor, isSameColorAs(Colors.black.withAlpha(0x40)));
    expect(lerpBNull75.surfaceTintColor, isSameColorAs(Colors.black.withAlpha(0x40)));
    expect(lerpBNull75.selectedShadowColor, isSameColorAs(Colors.black.withAlpha(0x40)));
    expect(lerpBNull75.showCheckmark, equals(true));
    expect(lerpBNull75.labelPadding, equals(const EdgeInsets.only(left: 2.0, right: 2.0)));
    expect(lerpBNull75.padding, equals(const EdgeInsets.all(1.0)));
    expect(lerpBNull75.side!.color, isSameColorAs(Colors.black.withAlpha(0xbf)));
    expect(lerpBNull75.shape, isA<StadiumBorder>());
    expect(lerpBNull75.labelStyle?.color, isSameColorAs(Colors.white.withAlpha(0x38)));
    expect(lerpBNull75.secondaryLabelStyle?.color, isSameColorAs(Colors.black.withAlpha(0x38)));
    expect(lerpBNull75.brightness, equals(Brightness.light));
    expect(lerpBNull75.elevation, 0.25);
    expect(lerpBNull75.pressElevation, 1.0);
    expect(lerpBNull75.checkmarkColor, isSameColorAs(Colors.black.withAlpha(0x40)));
    expect(lerpBNull75.iconTheme, const IconThemeData(size: 6.5));
  });

  testWidgets('Chip uses stateful color from chip theme', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode();

    const Color pressedColor = Color(0x00000001);
    const Color hoverColor = Color(0x00000002);
    const Color focusedColor = Color(0x00000003);
    const Color defaultColor = Color(0x00000004);
    const Color selectedColor = Color(0x00000005);
    const Color disabledColor = Color(0x00000006);

    Color getTextColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return disabledColor;
      }

      if (states.contains(MaterialState.pressed)) {
        return pressedColor;
      }

      if (states.contains(MaterialState.hovered)) {
        return hoverColor;
      }

      if (states.contains(MaterialState.focused)) {
        return focusedColor;
      }

      if (states.contains(MaterialState.selected)) {
        return selectedColor;
      }

      return defaultColor;
    }

    final TextStyle labelStyle = TextStyle(color: MaterialStateColor.resolveWith(getTextColor));
    Widget chipWidget({bool enabled = true, bool selected = false}) {
      return MaterialApp(
        theme: ThemeData(
          chipTheme: ThemeData().chipTheme.copyWith(
            labelStyle: labelStyle,
            secondaryLabelStyle: labelStyle,
          ),
        ),
        home: Scaffold(
          body: Focus(
            focusNode: focusNode,
            child: ChoiceChip(
              label: const Text('Chip'),
              selected: selected,
              onSelected: enabled ? (_) {} : null,
            ),
          ),
        ),
      );
    }

    Color textColor() {
      return tester.renderObject<RenderParagraph>(find.text('Chip')).text.style!.color!;
    }

    // Default, not disabled.
    await tester.pumpWidget(chipWidget());
    expect(textColor(), equals(defaultColor));

    // Selected.
    await tester.pumpWidget(chipWidget(selected: true));
    expect(textColor(), selectedColor);

    // Focused.
    final FocusNode chipFocusNode = focusNode.children.first;
    chipFocusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(textColor(), focusedColor);

    // Hovered.
    final Offset center = tester.getCenter(find.byType(ChoiceChip));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(textColor(), hoverColor);

    // Pressed.
    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(textColor(), pressedColor);

    // Disabled.
    await tester.pumpWidget(chipWidget(enabled: false));
    await tester.pumpAndSettle();
    expect(textColor(), disabledColor);

    focusNode.dispose();
  });

  testWidgets('Material2 - Chip uses stateful border side from resolveWith pattern', (
    WidgetTester tester,
  ) async {
    const Color selectedColor = Color(0x00000001);
    const Color defaultColor = Color(0x00000002);

    BorderSide getBorderSide(Set<MaterialState> states) {
      Color color = defaultColor;

      if (states.contains(MaterialState.selected)) {
        color = selectedColor;
      }

      return BorderSide(color: color);
    }

    Widget chipWidget({bool selected = false}) {
      return MaterialApp(
        theme: ThemeData(
          useMaterial3: false,
          chipTheme: ThemeData.light().chipTheme.copyWith(
            side: MaterialStateBorderSide.resolveWith(getBorderSide),
          ),
        ),
        home: Scaffold(
          body: ChoiceChip(label: const Text('Chip'), selected: selected, onSelected: (_) {}),
        ),
      );
    }

    // Default.
    await tester.pumpWidget(chipWidget());
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: defaultColor),
    );

    // Selected.
    await tester.pumpWidget(chipWidget(selected: true));
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: selectedColor),
    );
  });

  testWidgets('Material3 - Chip uses stateful border side from resolveWith pattern', (
    WidgetTester tester,
  ) async {
    const Color selectedColor = Color(0x00000001);
    const Color defaultColor = Color(0x00000002);

    BorderSide getBorderSide(Set<MaterialState> states) {
      Color color = defaultColor;

      if (states.contains(MaterialState.selected)) {
        color = selectedColor;
      }

      return BorderSide(color: color);
    }

    Widget chipWidget({bool selected = false}) {
      return MaterialApp(
        theme: ThemeData(
          chipTheme: ChipThemeData(side: MaterialStateBorderSide.resolveWith(getBorderSide)),
        ),
        home: Scaffold(
          body: ChoiceChip(label: const Text('Chip'), selected: selected, onSelected: (_) {}),
        ),
      );
    }

    // Default.
    await tester.pumpWidget(chipWidget());
    expect(find.byType(RawChip), paints..drrect(color: defaultColor));

    // Selected.
    await tester.pumpWidget(chipWidget(selected: true));
    expect(find.byType(RawChip), paints..drrect(color: selectedColor));
  });

  testWidgets('Material2 - Chip uses stateful border side from chip theme', (
    WidgetTester tester,
  ) async {
    const Color selectedColor = Color(0x00000001);
    const Color defaultColor = Color(0x00000002);

    BorderSide getBorderSide(Set<MaterialState> states) {
      Color color = defaultColor;
      if (states.contains(MaterialState.selected)) {
        color = selectedColor;
      }
      return BorderSide(color: color);
    }

    final ChipThemeData chipTheme = ChipThemeData.fromDefaults(
      brightness: Brightness.light,
      secondaryColor: Colors.blue,
      labelStyle: const TextStyle(),
    ).copyWith(side: _MaterialStateBorderSide(getBorderSide));

    Widget chipWidget({bool selected = false}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false, chipTheme: chipTheme),
        home: Scaffold(
          body: ChoiceChip(label: const Text('Chip'), selected: selected, onSelected: (_) {}),
        ),
      );
    }

    // Default.
    await tester.pumpWidget(chipWidget());
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: defaultColor),
    );

    // Selected.
    await tester.pumpWidget(chipWidget(selected: true));
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: selectedColor),
    );
  });

  testWidgets('Material3 - Chip uses stateful border side from chip theme', (
    WidgetTester tester,
  ) async {
    const Color selectedColor = Color(0x00000001);
    const Color defaultColor = Color(0x00000002);

    BorderSide getBorderSide(Set<MaterialState> states) {
      Color color = defaultColor;
      if (states.contains(MaterialState.selected)) {
        color = selectedColor;
      }
      return BorderSide(color: color);
    }

    final ChipThemeData chipTheme = ChipThemeData(side: _MaterialStateBorderSide(getBorderSide));

    Widget chipWidget({bool selected = false}) {
      return MaterialApp(
        theme: ThemeData(chipTheme: chipTheme),
        home: Scaffold(
          body: ChoiceChip(label: const Text('Chip'), selected: selected, onSelected: (_) {}),
        ),
      );
    }

    // Default.
    await tester.pumpWidget(chipWidget());
    expect(find.byType(RawChip), paints..drrect(color: defaultColor));

    // Selected.
    await tester.pumpWidget(chipWidget(selected: true));
    expect(find.byType(RawChip), paints..drrect(color: selectedColor));
  });

  testWidgets('Material2 - Chip uses stateful shape from chip theme', (WidgetTester tester) async {
    OutlinedBorder? getShape(Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        return const RoundedRectangleBorder();
      }
      return null;
    }

    final ChipThemeData chipTheme = ChipThemeData.fromDefaults(
      brightness: Brightness.light,
      secondaryColor: Colors.blue,
      labelStyle: const TextStyle(),
    ).copyWith(shape: _MaterialStateOutlinedBorder(getShape));

    Widget chipWidget({bool selected = false}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false, chipTheme: chipTheme),
        home: Scaffold(
          body: ChoiceChip(label: const Text('Chip'), selected: selected, onSelected: (_) {}),
        ),
      );
    }

    // Default.
    await tester.pumpWidget(chipWidget());
    expect(getMaterial(tester).shape, isA<StadiumBorder>());

    // Selected.
    await tester.pumpWidget(chipWidget(selected: true));
    expect(getMaterial(tester).shape, isA<RoundedRectangleBorder>());
  });

  testWidgets('Material3 - Chip uses stateful shape from chip theme', (WidgetTester tester) async {
    OutlinedBorder? getShape(Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        return const StadiumBorder();
      }
      return null;
    }

    final ChipThemeData chipTheme = ChipThemeData(shape: _MaterialStateOutlinedBorder(getShape));

    Widget chipWidget({bool selected = false}) {
      return MaterialApp(
        theme: ThemeData(chipTheme: chipTheme),
        home: Scaffold(
          body: ChoiceChip(label: const Text('Chip'), selected: selected, onSelected: (_) {}),
        ),
      );
    }

    // Default.
    await tester.pumpWidget(chipWidget());
    expect(getMaterial(tester).shape, isA<RoundedRectangleBorder>());

    // Selected.
    await tester.pumpWidget(chipWidget(selected: true));
    expect(getMaterial(tester).shape, isA<StadiumBorder>());
  });

  testWidgets('RawChip uses material state color from ChipTheme', (WidgetTester tester) async {
    const Color disabledSelectedColor = Color(0xffffff00);
    const Color disabledColor = Color(0xff00ff00);
    const Color backgroundColor = Color(0xff0000ff);
    const Color selectedColor = Color(0xffff0000);
    Widget buildApp({required bool enabled, required bool selected}) {
      return MaterialApp(
        theme: ThemeData(
          chipTheme: ChipThemeData(
            color: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
              if (states.contains(MaterialState.disabled) &&
                  states.contains(MaterialState.selected)) {
                return disabledSelectedColor;
              }
              if (states.contains(MaterialState.disabled)) {
                return disabledColor;
              }
              if (states.contains(MaterialState.selected)) {
                return selectedColor;
              }
              return backgroundColor;
            }),
          ),
        ),
        home: Material(
          child: RawChip(isEnabled: enabled, selected: selected, label: const Text('RawChip')),
        ),
      );
    }

    // Check theme color for enabled chip.
    await tester.pumpWidget(buildApp(enabled: true, selected: false));
    await tester.pumpAndSettle();

    // Enabled chip should have the provided backgroundColor.
    expect(getMaterialBox(tester), paints..rrect(color: backgroundColor));

    // Check theme color for disabled chip.
    await tester.pumpWidget(buildApp(enabled: false, selected: false));
    await tester.pumpAndSettle();

    // Disabled chip should have the provided disabledColor.
    expect(getMaterialBox(tester), paints..rrect(color: disabledColor));

    // Check theme color for enabled and selected chip.
    await tester.pumpWidget(buildApp(enabled: true, selected: true));
    await tester.pumpAndSettle();

    // Enabled & selected chip should have the provided selectedColor.
    expect(getMaterialBox(tester), paints..rrect(color: selectedColor));

    // Check theme color for disabled & selected chip.
    await tester.pumpWidget(buildApp(enabled: false, selected: true));
    await tester.pumpAndSettle();

    // Disabled & selected chip should have the provided disabledSelectedColor.
    expect(getMaterialBox(tester), paints..rrect(color: disabledSelectedColor));
  });

  testWidgets('RawChip uses state colors from ChipTheme', (WidgetTester tester) async {
    const ChipThemeData chipTheme = ChipThemeData(
      disabledColor: Color(0xadfefafe),
      backgroundColor: Color(0xcafefeed),
      selectedColor: Color(0xbeefcafe),
    );
    Widget buildApp({required bool enabled, required bool selected}) {
      return MaterialApp(
        theme: ThemeData(chipTheme: chipTheme),
        home: Material(
          child: RawChip(isEnabled: enabled, selected: selected, label: const Text('RawChip')),
        ),
      );
    }

    // Check theme color for enabled chip.
    await tester.pumpWidget(buildApp(enabled: true, selected: false));
    await tester.pumpAndSettle();

    // Enabled chip should have the provided backgroundColor.
    expect(getMaterialBox(tester), paints..rrect(color: chipTheme.backgroundColor));

    // Check theme color for disabled chip.
    await tester.pumpWidget(buildApp(enabled: false, selected: false));
    await tester.pumpAndSettle();

    // Disabled chip should have the provided disabledColor.
    expect(getMaterialBox(tester), paints..rrect(color: chipTheme.disabledColor));

    // Check theme color for enabled and selected chip.
    await tester.pumpWidget(buildApp(enabled: true, selected: true));
    await tester.pumpAndSettle();

    // Enabled & selected chip should have the provided selectedColor.
    expect(getMaterialBox(tester), paints..rrect(color: chipTheme.selectedColor));
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/119163.
  testWidgets('RawChip respects checkmark properties from ChipTheme', (WidgetTester tester) async {
    Widget buildRawChip({ChipThemeData? chipTheme}) {
      return MaterialApp(
        theme: ThemeData(chipTheme: chipTheme),
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: RawChip(
                selected: true,
                label: const SizedBox(width: 100, height: 100),
                onSelected: (bool newValue) {},
              ),
            ),
          ),
        ),
      );
    }

    // Test that the checkmark is painted.
    await tester.pumpWidget(
      buildRawChip(chipTheme: const ChipThemeData(checkmarkColor: Color(0xffff0000))),
    );

    RenderBox materialBox = getMaterialBox(tester);
    expect(materialBox, paints..path(color: const Color(0xffff0000), style: PaintingStyle.stroke));

    // Test that the checkmark is not painted when ChipThemeData.showCheckmark is false.
    await tester.pumpWidget(
      buildRawChip(
        chipTheme: const ChipThemeData(showCheckmark: false, checkmarkColor: Color(0xffff0000)),
      ),
    );
    await tester.pumpAndSettle();

    materialBox = getMaterialBox(tester);
    expect(
      materialBox,
      isNot(paints..path(color: const Color(0xffff0000), style: PaintingStyle.stroke)),
    );
  });

  testWidgets("Material3 - RawChip.shape's side is used when provided", (
    WidgetTester tester,
  ) async {
    Widget buildChip({OutlinedBorder? shape, BorderSide? side}) {
      return MaterialApp(
        theme: ThemeData(chipTheme: ChipThemeData(shape: shape, side: side)),
        home: const Material(child: Center(child: RawChip(label: Text('RawChip')))),
      );
    }

    // Test [RawChip.shape] with a side.
    await tester.pumpWidget(
      buildChip(
        shape: const RoundedRectangleBorder(
          side: BorderSide(color: Color(0xffff00ff)),
          borderRadius: BorderRadius.all(Radius.circular(7.0)),
        ),
      ),
    );

    // Chip should have the provided shape and the side from [RawChip.shape].
    expect(
      getMaterial(tester).shape,
      const RoundedRectangleBorder(
        side: BorderSide(color: Color(0xffff00ff)),
        borderRadius: BorderRadius.all(Radius.circular(7.0)),
      ),
    );

    // Test [RawChip.shape] with a side and [RawChip.side].
    await tester.pumpWidget(
      buildChip(
        shape: const RoundedRectangleBorder(
          side: BorderSide(color: Color(0xffff00ff)),
          borderRadius: BorderRadius.all(Radius.circular(7.0)),
        ),
        side: const BorderSide(color: Color(0xfffff000)),
      ),
    );
    await tester.pumpAndSettle();

    // Chip use shape from [RawChip.shape] and the side from [RawChip.side].
    // [RawChip.shape]'s side should be ignored.
    expect(
      getMaterial(tester).shape,
      const RoundedRectangleBorder(
        side: BorderSide(color: Color(0xfffff000)),
        borderRadius: BorderRadius.all(Radius.circular(7.0)),
      ),
    );
  });

  testWidgets('Material3 - ChipThemeData.iconTheme respects default iconTheme.size', (
    WidgetTester tester,
  ) async {
    Widget buildChip({IconThemeData? iconTheme}) {
      return MaterialApp(
        theme: ThemeData(chipTheme: ChipThemeData(iconTheme: iconTheme)),
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: RawChip(
                avatar: const Icon(Icons.add),
                label: const SizedBox(width: 100, height: 100),
                onSelected: (bool newValue) {},
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildChip(iconTheme: const IconThemeData(color: Color(0xff332211))));

    // Icon should have the default chip iconSize.
    expect(getIconData(tester).size, 18.0);
    expect(getIconData(tester).color, const Color(0xff332211));

    // Icon should have the provided iconSize.
    await tester.pumpWidget(
      buildChip(iconTheme: const IconThemeData(color: Color(0xff112233), size: 23.0)),
    );
    await tester.pumpAndSettle();

    expect(getIconData(tester).size, 23.0);
    expect(getIconData(tester).color, const Color(0xff112233));
  });

  testWidgets('ChipThemeData.avatarBoxConstraints updates avatar size constraints', (
    WidgetTester tester,
  ) async {
    const double border = 1.0;
    const double iconSize = 18.0;
    const double labelPadding = 8.0;
    const double padding = 8.0;
    const Size labelSize = Size(75, 75);

    // Test default avatar layout constraints.
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          chipTheme: const ChipThemeData(avatarBoxConstraints: BoxConstraints.tightForFinite()),
        ),
        home: Material(
          child: Center(
            child: RawChip(
              avatar: const Icon(Icons.favorite),
              label: Container(
                width: labelSize.width,
                height: labelSize.width,
                color: const Color(0xFFFF0000),
              ),
            ),
          ),
        ),
      ),
    );

    expect(tester.getSize(find.byType(RawChip)).width, equals(127.0));
    expect(tester.getSize(find.byType(RawChip)).height, equals(93.0));

    // Calculate the distance between avatar and chip edges.
    final Offset chipTopLeft = tester.getTopLeft(find.byWidget(getMaterial(tester)));
    final Offset avatarCenter = tester.getCenter(find.byIcon(Icons.favorite));
    expect(chipTopLeft.dx, avatarCenter.dx - (iconSize / 2) - padding - border);
    expect(chipTopLeft.dy, avatarCenter.dy - (labelSize.width / 2) - padding - border);

    // Calculate the distance between avatar and label.
    final Offset labelTopLeft = tester.getTopLeft(find.byType(Container));
    expect(labelTopLeft.dx, avatarCenter.dx + (iconSize / 2) + labelPadding);
  });

  testWidgets('ChipThemeData.deleteIconBoxConstraints updates delete icon size constraints', (
    WidgetTester tester,
  ) async {
    const double border = 1.0;
    const double iconSize = 18.0;
    const double labelPadding = 8.0;
    const double padding = 8.0;
    const Size labelSize = Size(75, 75);

    // Test custom delete layout constraints.
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          chipTheme: const ChipThemeData(deleteIconBoxConstraints: BoxConstraints.tightForFinite()),
        ),
        home: Material(
          child: Center(
            child: RawChip(
              onDeleted: () {},
              label: Container(
                width: labelSize.width,
                height: labelSize.width,
                color: const Color(0xFFFF0000),
              ),
            ),
          ),
        ),
      ),
    );

    expect(tester.getSize(find.byType(RawChip)).width, equals(127.0));
    expect(tester.getSize(find.byType(RawChip)).height, equals(93.0));

    // Calculate the distance between delete icon and chip edges.
    final Offset chipTopRight = tester.getTopRight(find.byWidget(getMaterial(tester)));
    final Offset deleteIconCenter = tester.getCenter(find.byIcon(Icons.cancel));
    expect(chipTopRight.dx, deleteIconCenter.dx + (iconSize / 2) + padding + border);
    expect(chipTopRight.dy, deleteIconCenter.dy - (labelSize.width / 2) - padding - border);

    // Calculate the distance between delete icon and label.
    final Offset labelTopRight = tester.getTopRight(find.byType(Container));
    expect(labelTopRight.dx, deleteIconCenter.dx - (iconSize / 2) - labelPadding);
  });

  testWidgets('ChipThemeData.iconTheme updates avatar and delete icons', (
    WidgetTester tester,
  ) async {
    const Color iconColor = Color(0xffff0000);
    const double iconSize = 32.0;
    const IconData avatarIcon = Icons.favorite;
    const IconData deleteIcon = Icons.delete;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          chipTheme: const ChipThemeData(
            iconTheme: IconThemeData(color: iconColor, size: iconSize),
          ),
        ),
        home: Material(
          child: Center(
            child: RawChip(
              avatar: const Icon(Icons.favorite),
              deleteIcon: const Icon(Icons.delete),
              onDeleted: () {},
              label: const SizedBox(height: 100),
            ),
          ),
        ),
      ),
    );

    // Test rendered icon size.
    final RenderBox avatarIconBox = tester.renderObject(find.byIcon(avatarIcon));
    final RenderBox deleteIconBox = tester.renderObject(find.byIcon(deleteIcon));
    expect(avatarIconBox.size.width, equals(iconSize));
    expect(deleteIconBox.size.width, equals(iconSize));

    // Test rendered icon color.
    expect(getIconStyle(tester, avatarIcon)?.color, iconColor);
    expect(getIconStyle(tester, deleteIcon)?.color, iconColor);
  });

  testWidgets('ChipThemeData.deleteIconColor overrides ChipThemeData.iconTheme color', (
    WidgetTester tester,
  ) async {
    const Color iconColor = Color(0xffff00ff);
    const Color deleteIconColor = Color(0xffff00ff);
    const IconData deleteIcon = Icons.delete;

    Widget buildChip({Color? deleteIconColor, Color? iconColor}) {
      return MaterialApp(
        theme: ThemeData(
          chipTheme: ChipThemeData(
            deleteIconColor: deleteIconColor,
            iconTheme: IconThemeData(color: iconColor),
          ),
        ),
        home: Material(
          child: Center(
            child: RawChip(
              deleteIcon: const Icon(Icons.delete),
              onDeleted: () {},
              label: const SizedBox(height: 100),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildChip(iconColor: iconColor));

    // Test rendered icon color.
    expect(getIconStyle(tester, deleteIcon)?.color, iconColor);

    await tester.pumpWidget(buildChip(deleteIconColor: deleteIconColor, iconColor: iconColor));

    // Test rendered icon color.
    expect(getIconStyle(tester, deleteIcon)?.color, deleteIconColor);
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/135136.
  testWidgets('MaterialStateBorderSide properly lerp in ChipThemeData.side', (
    WidgetTester tester,
  ) async {
    late ColorScheme colorScheme;

    Widget buildChip({required Color seedColor}) {
      colorScheme = ColorScheme.fromSeed(seedColor: seedColor);
      return MaterialApp(
        theme: ThemeData(
          colorScheme: colorScheme,
          chipTheme: ChipThemeData(
            side: MaterialStateBorderSide.resolveWith((Set<MaterialState> states) {
              return BorderSide(color: colorScheme.primary, width: 4.0);
            }),
          ),
        ),
        home: const Scaffold(body: RawChip(label: Text('Chip'))),
      );
    }

    await tester.pumpWidget(buildChip(seedColor: Colors.red));
    await tester.pumpAndSettle();

    RenderBox getChipRenderBox() {
      return tester.renderObject<RenderBox>(find.byType(RawChip));
    }

    expect(getChipRenderBox(), paints..drrect(color: colorScheme.primary));

    await tester.pumpWidget(buildChip(seedColor: Colors.blue));
    await tester.pump(kPressTimeout);

    expect(getChipRenderBox(), paints..drrect(color: colorScheme.primary));
  });
}

class _MaterialStateOutlinedBorder extends StadiumBorder implements MaterialStateOutlinedBorder {
  const _MaterialStateOutlinedBorder(this.resolver);

  final MaterialPropertyResolver<OutlinedBorder?> resolver;

  @override
  OutlinedBorder? resolve(Set<MaterialState> states) => resolver(states);
}

class _MaterialStateBorderSide extends MaterialStateBorderSide {
  const _MaterialStateBorderSide(this.resolver);

  final MaterialPropertyResolver<BorderSide?> resolver;

  @override
  BorderSide? resolve(Set<MaterialState> states) => resolver(states);
}
