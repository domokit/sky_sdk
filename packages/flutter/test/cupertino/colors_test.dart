// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

class DependentWidget extends StatelessWidget {
  const DependentWidget({super.key, required this.color});

  final Color color;

  @override
  Widget build(BuildContext context) {
    final Color resolved = CupertinoDynamicColor.resolve(color, context);
    return DecoratedBox(decoration: BoxDecoration(color: resolved), child: const SizedBox.expand());
  }
}

const Color color0 = Color(0xFF000000);
const Color color1 = Color(0xFF000001);
const Color color2 = Color(0xFF000002);
const Color color3 = Color(0xFF000003);
const Color color4 = Color(0xFF000004);
const Color color5 = Color(0xFF000005);
const Color color6 = Color(0xFF000006);
const Color color7 = Color(0xFF000007);

// A color that depends on color vibrancy, accessibility contrast, as well as user
// interface elevation.
const CupertinoDynamicColor dynamicColor = CupertinoDynamicColor(
  color: color0,
  darkColor: color1,
  elevatedColor: color2,
  highContrastColor: color3,
  darkElevatedColor: color4,
  darkHighContrastColor: color5,
  highContrastElevatedColor: color6,
  darkHighContrastElevatedColor: color7,
);

// A color that uses [color0] in every circumstance.
const Color notSoDynamicColor1 = CupertinoDynamicColor(
  color: color0,
  darkColor: color0,
  darkHighContrastColor: color0,
  darkElevatedColor: color0,
  darkHighContrastElevatedColor: color0,
  highContrastColor: color0,
  highContrastElevatedColor: color0,
  elevatedColor: color0,
);

// A color that uses [color1] for light mode, and [color0] for dark mode.
const Color vibrancyDependentColor1 = CupertinoDynamicColor(
  color: color1,
  elevatedColor: color1,
  highContrastColor: color1,
  highContrastElevatedColor: color1,
  darkColor: color0,
  darkHighContrastColor: color0,
  darkElevatedColor: color0,
  darkHighContrastElevatedColor: color0,
);

// A color that uses [color1] for normal contrast mode, and [color0] for high
// contrast mode.
const Color contrastDependentColor1 = CupertinoDynamicColor(
  color: color1,
  darkColor: color1,
  elevatedColor: color1,
  darkElevatedColor: color1,
  highContrastColor: color0,
  darkHighContrastColor: color0,
  highContrastElevatedColor: color0,
  darkHighContrastElevatedColor: color0,
);

// A color that uses [color1] for base interface elevation, and [color0] for elevated
// interface elevation.
const Color elevationDependentColor1 = CupertinoDynamicColor(
  color: color1,
  darkColor: color1,
  highContrastColor: color1,
  darkHighContrastColor: color1,
  elevatedColor: color0,
  darkElevatedColor: color0,
  highContrastElevatedColor: color0,
  darkHighContrastElevatedColor: color0,
);

void main() {
  test('== works as expected', () {
    expect(
      dynamicColor,
      const CupertinoDynamicColor(
        color: color0,
        darkColor: color1,
        elevatedColor: color2,
        highContrastColor: color3,
        darkElevatedColor: color4,
        darkHighContrastColor: color5,
        highContrastElevatedColor: color6,
        darkHighContrastElevatedColor: color7,
      ),
    );

    expect(notSoDynamicColor1, isNot(vibrancyDependentColor1));

    expect(notSoDynamicColor1, isNot(contrastDependentColor1));

    expect(
      vibrancyDependentColor1,
      isNot(
        const CupertinoDynamicColor(
          color: color0,
          elevatedColor: color0,
          highContrastColor: color0,
          highContrastElevatedColor: color0,
          darkColor: color0,
          darkHighContrastColor: color0,
          darkElevatedColor: color0,
          darkHighContrastElevatedColor: color0,
        ),
      ),
    );
  });

  test('CupertinoDynamicColor.toString() works', () {
    expect(
      dynamicColor.toString(),
      contains(
        'CupertinoDynamicColor(*color = ${const Color(0xff000000)}*, '
        'darkColor = ${const Color(0xff000001)}, '
        'highContrastColor = ${const Color(0xff000003)}, '
        'darkHighContrastColor = ${const Color(0xff000005)}, '
        'elevatedColor = ${const Color(0xff000002)}, '
        'darkElevatedColor = ${const Color(0xff000004)}, '
        'highContrastElevatedColor = ${const Color(0xff000006)}, '
        'darkHighContrastElevatedColor = ${const Color(0xff000007)}',
      ),
    );
    expect(
      notSoDynamicColor1.toString(),
      contains('CupertinoDynamicColor(*color = ${const Color(0xff000000)}*'),
    );
    expect(
      vibrancyDependentColor1.toString(),
      contains(
        'CupertinoDynamicColor(*color = ${const Color(0xff000001)}*, darkColor = ${const Color(0xff000000)}',
      ),
    );
    expect(
      contrastDependentColor1.toString(),
      contains(
        'CupertinoDynamicColor(*color = ${const Color(0xff000001)}*, highContrastColor = ${const Color(0xff000000)}',
      ),
    );
    expect(
      elevationDependentColor1.toString(),
      contains(
        'CupertinoDynamicColor(*color = ${const Color(0xff000001)}*, elevatedColor = ${const Color(0xff000000)}',
      ),
    );

    expect(
      const CupertinoDynamicColor.withBrightnessAndContrast(
        color: color0,
        darkColor: color1,
        highContrastColor: color2,
        darkHighContrastColor: color3,
      ).toString(),
      contains(
        'CupertinoDynamicColor(*color = ${const Color(0xff000000)}*, '
        'darkColor = ${const Color(0xff000001)}, '
        'highContrastColor = ${const Color(0xff000002)}, '
        'darkHighContrastColor = ${const Color(0xff000003)}',
      ),
    );
  });

  test('can resolve null color', () {
    expect(CupertinoDynamicColor.maybeResolve(null, _NullElement.instance), isNull);
  });

  test('withVibrancy constructor creates colors that may depend on vibrancy', () {
    expect(
      vibrancyDependentColor1,
      const CupertinoDynamicColor.withBrightness(color: color1, darkColor: color0),
    );
  });

  test(
    'withVibrancyAndContrast constructor creates colors that may depend on contrast and vibrancy',
    () {
      expect(
        contrastDependentColor1,
        const CupertinoDynamicColor.withBrightnessAndContrast(
          color: color1,
          darkColor: color1,
          highContrastColor: color0,
          darkHighContrastColor: color0,
        ),
      );

      expect(
        const CupertinoDynamicColor(
          color: color0,
          darkColor: color1,
          highContrastColor: color2,
          darkHighContrastColor: color3,
          elevatedColor: color0,
          darkElevatedColor: color1,
          highContrastElevatedColor: color2,
          darkHighContrastElevatedColor: color3,
        ),
        const CupertinoDynamicColor.withBrightnessAndContrast(
          color: color0,
          darkColor: color1,
          highContrastColor: color2,
          darkHighContrastColor: color3,
        ),
      );
    },
  );

  testWidgets('Dynamic colors that are not actually dynamic should not claim dependencies', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const DependentWidget(color: notSoDynamicColor1));

    expect(tester.takeException(), null);
    expect(find.byType(DependentWidget), paints..rect(color: color0));
  });

  testWidgets(
    'Dynamic colors that are only dependent on vibrancy should not claim unnecessary dependencies, '
    'and its resolved color should change when its dependency changes',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MediaQuery(
          data: MediaQueryData(),
          child: DependentWidget(color: vibrancyDependentColor1),
        ),
      );

      expect(tester.takeException(), null);
      expect(find.byType(DependentWidget), paints..rect(color: color1));
      expect(find.byType(DependentWidget), isNot(paints..rect(color: color0)));

      // Changing color vibrancy works.
      await tester.pumpWidget(
        const MediaQuery(
          data: MediaQueryData(platformBrightness: Brightness.dark),
          child: DependentWidget(color: vibrancyDependentColor1),
        ),
      );

      expect(tester.takeException(), null);
      expect(find.byType(DependentWidget), paints..rect(color: color0));
      expect(find.byType(DependentWidget), isNot(paints..rect(color: color1)));

      // CupertinoTheme should take precedence over MediaQuery.
      await tester.pumpWidget(
        const CupertinoTheme(
          data: CupertinoThemeData(brightness: Brightness.light),
          child: MediaQuery(
            data: MediaQueryData(platformBrightness: Brightness.dark),
            child: DependentWidget(color: vibrancyDependentColor1),
          ),
        ),
      );

      expect(tester.takeException(), null);
      expect(find.byType(DependentWidget), paints..rect(color: color1));
      expect(find.byType(DependentWidget), isNot(paints..rect(color: color0)));
    },
  );

  testWidgets(
    'Dynamic colors that are only dependent on accessibility contrast should not claim unnecessary dependencies, '
    'and its resolved color should change when its dependency changes',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MediaQuery(
          data: MediaQueryData(),
          child: DependentWidget(color: contrastDependentColor1),
        ),
      );

      expect(tester.takeException(), null);
      expect(find.byType(DependentWidget), paints..rect(color: color1));
      expect(find.byType(DependentWidget), isNot(paints..rect(color: color0)));

      // Changing accessibility contrast works.
      await tester.pumpWidget(
        const MediaQuery(
          data: MediaQueryData(highContrast: true),
          child: DependentWidget(color: contrastDependentColor1),
        ),
      );

      expect(tester.takeException(), null);
      expect(find.byType(DependentWidget), paints..rect(color: color0));
      expect(find.byType(DependentWidget), isNot(paints..rect(color: color1)));
    },
  );

  testWidgets(
    'Dynamic colors that are only dependent on elevation level should not claim unnecessary dependencies, '
    'and its resolved color should change when its dependency changes',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const CupertinoUserInterfaceLevel(
          data: CupertinoUserInterfaceLevelData.base,
          child: DependentWidget(color: elevationDependentColor1),
        ),
      );

      expect(tester.takeException(), null);
      expect(find.byType(DependentWidget), paints..rect(color: color1));
      expect(find.byType(DependentWidget), isNot(paints..rect(color: color0)));

      // Changing UI elevation works.
      await tester.pumpWidget(
        const CupertinoUserInterfaceLevel(
          data: CupertinoUserInterfaceLevelData.elevated,
          child: DependentWidget(color: elevationDependentColor1),
        ),
      );

      expect(tester.takeException(), null);
      expect(find.byType(DependentWidget), paints..rect(color: color0));
      expect(find.byType(DependentWidget), isNot(paints..rect(color: color1)));
    },
  );

  testWidgets('Dynamic color with all 3 dependencies works', (WidgetTester tester) async {
    const Color dynamicRainbowColor1 = CupertinoDynamicColor(
      color: color0,
      darkColor: color1,
      highContrastColor: color2,
      darkHighContrastColor: color3,
      darkElevatedColor: color4,
      highContrastElevatedColor: color5,
      darkHighContrastElevatedColor: color6,
      elevatedColor: color7,
    );

    await tester.pumpWidget(
      const MediaQuery(
        data: MediaQueryData(),
        child: CupertinoUserInterfaceLevel(
          data: CupertinoUserInterfaceLevelData.base,
          child: DependentWidget(color: dynamicRainbowColor1),
        ),
      ),
    );
    expect(find.byType(DependentWidget), paints..rect(color: color0));

    await tester.pumpWidget(
      const MediaQuery(
        data: MediaQueryData(platformBrightness: Brightness.dark),
        child: CupertinoUserInterfaceLevel(
          data: CupertinoUserInterfaceLevelData.base,
          child: DependentWidget(color: dynamicRainbowColor1),
        ),
      ),
    );
    expect(find.byType(DependentWidget), paints..rect(color: color1));

    await tester.pumpWidget(
      const MediaQuery(
        data: MediaQueryData(highContrast: true),
        child: CupertinoUserInterfaceLevel(
          data: CupertinoUserInterfaceLevelData.base,
          child: DependentWidget(color: dynamicRainbowColor1),
        ),
      ),
    );
    expect(find.byType(DependentWidget), paints..rect(color: color2));

    await tester.pumpWidget(
      const MediaQuery(
        data: MediaQueryData(platformBrightness: Brightness.dark, highContrast: true),
        child: CupertinoUserInterfaceLevel(
          data: CupertinoUserInterfaceLevelData.base,
          child: DependentWidget(color: dynamicRainbowColor1),
        ),
      ),
    );
    expect(find.byType(DependentWidget), paints..rect(color: color3));

    await tester.pumpWidget(
      const MediaQuery(
        data: MediaQueryData(platformBrightness: Brightness.dark),
        child: CupertinoUserInterfaceLevel(
          data: CupertinoUserInterfaceLevelData.elevated,
          child: DependentWidget(color: dynamicRainbowColor1),
        ),
      ),
    );
    expect(find.byType(DependentWidget), paints..rect(color: color4));

    await tester.pumpWidget(
      const MediaQuery(
        data: MediaQueryData(highContrast: true),
        child: CupertinoUserInterfaceLevel(
          data: CupertinoUserInterfaceLevelData.elevated,
          child: DependentWidget(color: dynamicRainbowColor1),
        ),
      ),
    );
    expect(find.byType(DependentWidget), paints..rect(color: color5));

    await tester.pumpWidget(
      const MediaQuery(
        data: MediaQueryData(platformBrightness: Brightness.dark, highContrast: true),
        child: CupertinoUserInterfaceLevel(
          data: CupertinoUserInterfaceLevelData.elevated,
          child: DependentWidget(color: dynamicRainbowColor1),
        ),
      ),
    );
    expect(find.byType(DependentWidget), paints..rect(color: color6));

    await tester.pumpWidget(
      const MediaQuery(
        data: MediaQueryData(),
        child: CupertinoUserInterfaceLevel(
          data: CupertinoUserInterfaceLevelData.elevated,
          child: DependentWidget(color: dynamicRainbowColor1),
        ),
      ),
    );
    expect(find.byType(DependentWidget), paints..rect(color: color7));
  });

  testWidgets('CupertinoDynamicColor toARGB32() is the same as value', (WidgetTester tester) async {
    late CupertinoDynamicColor color;
    await tester.pumpWidget(
      CupertinoApp(
        theme: const CupertinoThemeData(brightness: Brightness.dark, primaryColor: dynamicColor),
        home: Builder(
          builder: (BuildContext context) {
            color = CupertinoTheme.of(context).primaryColor as CupertinoDynamicColor;
            return const Placeholder();
          },
        ),
      ),
    );

    expect(color.value, color.toARGB32());
  });

  testWidgets('CupertinoDynamicColor used in a CupertinoTheme', (WidgetTester tester) async {
    late CupertinoDynamicColor color;
    await tester.pumpWidget(
      CupertinoApp(
        theme: const CupertinoThemeData(brightness: Brightness.dark, primaryColor: dynamicColor),
        home: Builder(
          builder: (BuildContext context) {
            color = CupertinoTheme.of(context).primaryColor as CupertinoDynamicColor;
            return const Placeholder();
          },
        ),
      ),
    );

    expect(color.value, dynamicColor.darkColor.value);

    // Changing dependencies works.
    await tester.pumpWidget(
      CupertinoApp(
        theme: const CupertinoThemeData(brightness: Brightness.light, primaryColor: dynamicColor),
        home: Builder(
          builder: (BuildContext context) {
            color = CupertinoTheme.of(context).primaryColor as CupertinoDynamicColor;
            return const Placeholder();
          },
        ),
      ),
    );

    expect(color.value, dynamicColor.color.value);

    // Having a dependency below the CupertinoTheme widget works.
    await tester.pumpWidget(
      CupertinoApp(
        theme: const CupertinoThemeData(primaryColor: dynamicColor),
        home: MediaQuery(
          data: const MediaQueryData(),
          child: CupertinoUserInterfaceLevel(
            data: CupertinoUserInterfaceLevelData.base,
            child: Builder(
              builder: (BuildContext context) {
                color = CupertinoTheme.of(context).primaryColor as CupertinoDynamicColor;
                return const Placeholder();
              },
            ),
          ),
        ),
      ),
    );

    expect(color.value, dynamicColor.color.value);

    // Changing dependencies works.
    await tester.pumpWidget(
      CupertinoApp(
        // No brightness is explicitly specified here so it should defer to MediaQuery.
        theme: const CupertinoThemeData(primaryColor: dynamicColor),
        home: MediaQuery(
          data: const MediaQueryData(platformBrightness: Brightness.dark, highContrast: true),
          child: CupertinoUserInterfaceLevel(
            data: CupertinoUserInterfaceLevelData.elevated,
            child: Builder(
              builder: (BuildContext context) {
                color = CupertinoTheme.of(context).primaryColor as CupertinoDynamicColor;
                return const Placeholder();
              },
            ),
          ),
        ),
      ),
    );

    expect(color.value, dynamicColor.darkHighContrastElevatedColor.value);
  });

  group('MaterialApp:', () {
    Color? color;
    setUp(() {
      color = null;
    });

    testWidgets('dynamic color works in cupertino override theme', (WidgetTester tester) async {
      CupertinoDynamicColor typedColor() => color! as CupertinoDynamicColor;

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            cupertinoOverrideTheme: const CupertinoThemeData(
              brightness: Brightness.dark,
              primaryColor: dynamicColor,
            ),
          ),
          home: MediaQuery(
            data: const MediaQueryData(),
            child: CupertinoUserInterfaceLevel(
              data: CupertinoUserInterfaceLevelData.base,
              child: Builder(
                builder: (BuildContext context) {
                  color = CupertinoTheme.of(context).primaryColor;
                  return const Placeholder();
                },
              ),
            ),
          ),
        ),
      );

      // Explicit brightness is respected.
      expect(typedColor().value, dynamicColor.darkColor.value);
      color = null;

      // Changing dependencies works.
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            cupertinoOverrideTheme: const CupertinoThemeData(
              brightness: Brightness.dark,
              primaryColor: dynamicColor,
            ),
          ),
          home: MediaQuery(
            data: const MediaQueryData(platformBrightness: Brightness.dark, highContrast: true),
            child: CupertinoUserInterfaceLevel(
              data: CupertinoUserInterfaceLevelData.elevated,
              child: Builder(
                builder: (BuildContext context) {
                  color = CupertinoTheme.of(context).primaryColor;
                  return const Placeholder();
                },
              ),
            ),
          ),
        ),
      );

      expect(typedColor().value, dynamicColor.darkHighContrastElevatedColor.value);
    });

    testWidgets('dynamic color does not work in a material theme', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          // This will create a MaterialBasedCupertinoThemeData with primaryColor set to `dynamicColor`.
          theme: ThemeData(colorScheme: const ColorScheme.dark(primary: dynamicColor)),
          home: MediaQuery(
            data: const MediaQueryData(platformBrightness: Brightness.dark, highContrast: true),
            child: CupertinoUserInterfaceLevel(
              data: CupertinoUserInterfaceLevelData.elevated,
              child: Builder(
                builder: (BuildContext context) {
                  color = CupertinoTheme.of(context).primaryColor;
                  return const Placeholder();
                },
              ),
            ),
          ),
        ),
      );

      // The color is not resolved.
      expect(color, dynamicColor);
      expect(color, isNot(dynamicColor.darkHighContrastElevatedColor));
    });
  });
}

class _NullElement extends Element {
  _NullElement() : super(const _NullWidget());

  static _NullElement instance = _NullElement();

  @override
  bool get debugDoingBuild => throw UnimplementedError();
}

class _NullWidget extends Widget {
  const _NullWidget();

  @override
  Element createElement() => throw UnimplementedError();
}
