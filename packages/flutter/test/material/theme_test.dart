// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;
import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  const TextTheme defaultGeometryTheme = Typography.englishLike2014;
  const TextTheme defaultGeometryThemeM3 = Typography.englishLike2021;

  test('ThemeDataTween control test', () {
    final ThemeData light = ThemeData.light();
    final ThemeData dark = ThemeData.dark();
    final ThemeDataTween tween = ThemeDataTween(begin: light, end: dark);
    expect(tween.lerp(0.25), equals(ThemeData.lerp(light, dark, 0.25)));
  });

  testWidgets('PopupMenu inherits app theme', (WidgetTester tester) async {
    final Key popupMenuButtonKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(brightness: Brightness.dark),
        home: Scaffold(
          appBar: AppBar(
            actions: <Widget>[
              PopupMenuButton<String>(
                key: popupMenuButtonKey,
                itemBuilder: (BuildContext context) {
                  return <PopupMenuItem<String>>[
                    const PopupMenuItem<String>(child: Text('menuItem')),
                  ];
                },
              ),
            ],
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(popupMenuButtonKey));
    await tester.pump(const Duration(seconds: 1));

    expect(Theme.of(tester.element(find.text('menuItem'))).brightness, equals(Brightness.dark));
  });

  testWidgets('Theme overrides selection style', (WidgetTester tester) async {
    final Key key = UniqueKey();
    const Color defaultSelectionColor = Color(0x11111111);
    const Color defaultCursorColor = Color(0x22222222);
    const Color themeSelectionColor = Color(0x33333333);
    const Color themeCursorColor = Color(0x44444444);
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(brightness: Brightness.dark),
        home: Scaffold(
          body: DefaultSelectionStyle(
            selectionColor: defaultSelectionColor,
            cursorColor: defaultCursorColor,
            child: Theme(
              data: ThemeData(
                textSelectionTheme: const TextSelectionThemeData(
                  selectionColor: themeSelectionColor,
                  cursorColor: themeCursorColor,
                ),
              ),
              child: TextField(key: key),
            ),
          ),
        ),
      ),
    );
    // Finds RenderEditable.
    final RenderObject root = tester.renderObject(find.byType(EditableText));
    late RenderEditable renderEditable;
    void recursiveFinder(RenderObject child) {
      if (child is RenderEditable) {
        renderEditable = child;
        return;
      }
      child.visitChildren(recursiveFinder);
    }

    root.visitChildren(recursiveFinder);

    // Focus text field so it has a selection color. The selection color is null
    // on an unfocused text field.
    await tester.tap(find.byKey(key));
    await tester.pump();

    expect(renderEditable.selectionColor, themeSelectionColor);
    expect(tester.widget<EditableText>(find.byType(EditableText)).cursorColor, themeCursorColor);
  });

  testWidgets('Material2 - Fallback theme', (WidgetTester tester) async {
    late BuildContext capturedContext;
    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: Builder(
          builder: (BuildContext context) {
            capturedContext = context;
            return Container();
          },
        ),
      ),
    );

    expect(
      Theme.of(capturedContext),
      equals(ThemeData.localize(ThemeData.fallback(useMaterial3: false), defaultGeometryTheme)),
    );
  });

  testWidgets('Material3 - Fallback theme', (WidgetTester tester) async {
    late BuildContext capturedContextM3;
    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: true),
        child: Builder(
          builder: (BuildContext context) {
            capturedContextM3 = context;
            return Container();
          },
        ),
      ),
    );

    expect(
      Theme.of(capturedContextM3),
      equals(ThemeData.localize(ThemeData.fallback(useMaterial3: true), defaultGeometryThemeM3)),
    );
  });

  testWidgets('ThemeData.localize memoizes the result', (WidgetTester tester) async {
    final ThemeData light = ThemeData.light();
    final ThemeData dark = ThemeData.dark();

    // Same input, same output.
    expect(
      ThemeData.localize(light, defaultGeometryTheme),
      same(ThemeData.localize(light, defaultGeometryTheme)),
    );

    // Different text geometry, different output.
    expect(
      ThemeData.localize(light, defaultGeometryTheme),
      isNot(same(ThemeData.localize(light, Typography.tall2014))),
    );

    // Different base theme, different output.
    expect(
      ThemeData.localize(light, defaultGeometryTheme),
      isNot(same(ThemeData.localize(dark, defaultGeometryTheme))),
    );
  });

  testWidgets('Material2 - ThemeData with null typography uses proper defaults', (
    WidgetTester tester,
  ) async {
    final ThemeData m2Theme = ThemeData(useMaterial3: false);
    expect(m2Theme.typography, Typography.material2014());
  });

  testWidgets('Material3 - ThemeData with null typography uses proper defaults', (
    WidgetTester tester,
  ) async {
    final ThemeData m3Theme = ThemeData(useMaterial3: true);
    expect(m3Theme.typography, Typography.material2021(colorScheme: m3Theme.colorScheme));
  });

  testWidgets('PopupMenu inherits shadowed app theme', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/5572
    final Key popupMenuButtonKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(brightness: Brightness.dark),
        home: Theme(
          data: ThemeData(brightness: Brightness.light),
          child: Scaffold(
            appBar: AppBar(
              actions: <Widget>[
                PopupMenuButton<String>(
                  key: popupMenuButtonKey,
                  itemBuilder: (BuildContext context) {
                    return <PopupMenuItem<String>>[
                      const PopupMenuItem<String>(child: Text('menuItem')),
                    ];
                  },
                ),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(popupMenuButtonKey));
    await tester.pump(const Duration(seconds: 1));

    expect(Theme.of(tester.element(find.text('menuItem'))).brightness, equals(Brightness.light));
  });

  testWidgets('DropdownMenu inherits shadowed app theme', (WidgetTester tester) async {
    final Key dropdownMenuButtonKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(brightness: Brightness.dark),
        home: Theme(
          data: ThemeData(brightness: Brightness.light),
          child: Scaffold(
            appBar: AppBar(
              actions: <Widget>[
                DropdownButton<String>(
                  key: dropdownMenuButtonKey,
                  onChanged: (String? newValue) {},
                  value: 'menuItem',
                  items: const <DropdownMenuItem<String>>[
                    DropdownMenuItem<String>(value: 'menuItem', child: Text('menuItem')),
                  ],
                ),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(dropdownMenuButtonKey));
    await tester.pump(const Duration(seconds: 1));

    for (final Element item in tester.elementList(find.text('menuItem'))) {
      expect(Theme.of(item).brightness, equals(Brightness.light));
    }
  });

  testWidgets('ModalBottomSheet inherits shadowed app theme', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(brightness: Brightness.dark),
        home: Theme(
          data: ThemeData(brightness: Brightness.light),
          child: Scaffold(
            body: Center(
              child: Builder(
                builder: (BuildContext context) {
                  return ElevatedButton(
                    onPressed: () {
                      showModalBottomSheet<void>(
                        context: context,
                        builder: (BuildContext context) => const Text('bottomSheet'),
                      );
                    },
                    child: const Text('SHOW'),
                  );
                },
              ),
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('SHOW'));
    await tester.pump(); // start animation
    await tester.pump(const Duration(seconds: 1)); // end animation
    expect(Theme.of(tester.element(find.text('bottomSheet'))).brightness, equals(Brightness.light));
  });

  testWidgets('Dialog inherits shadowed app theme', (WidgetTester tester) async {
    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(brightness: Brightness.dark),
        home: Theme(
          data: ThemeData(brightness: Brightness.light),
          child: Scaffold(
            key: scaffoldKey,
            body: Center(
              child: Builder(
                builder: (BuildContext context) {
                  return ElevatedButton(
                    onPressed: () {
                      showDialog<void>(
                        context: context,
                        builder: (BuildContext context) => const Text('dialog'),
                      );
                    },
                    child: const Text('SHOW'),
                  );
                },
              ),
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('SHOW'));
    await tester.pump(const Duration(seconds: 1));
    expect(Theme.of(tester.element(find.text('dialog'))).brightness, equals(Brightness.light));
  });

  testWidgets("Scaffold inherits theme's scaffoldBackgroundColor", (WidgetTester tester) async {
    const Color green = Color(0xFF00FF00);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(scaffoldBackgroundColor: green),
        home: Scaffold(
          body: Center(
            child: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  onTap: () {
                    showDialog<void>(
                      context: context,
                      builder: (BuildContext context) {
                        return const Scaffold(body: SizedBox(width: 200.0, height: 200.0));
                      },
                    );
                  },
                  child: const Text('SHOW'),
                );
              },
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('SHOW'));
    await tester.pump(const Duration(seconds: 1));

    final List<Material> materials = tester.widgetList<Material>(find.byType(Material)).toList();
    expect(materials.length, equals(2));
    expect(materials[0].color, green); // app scaffold
    expect(materials[1].color, green); // dialog scaffold
  });

  testWidgets('IconThemes are applied', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(iconTheme: const IconThemeData(color: Colors.green, size: 10.0)),
        home: const Icon(Icons.computer),
      ),
    );

    RenderParagraph glyphText = tester.renderObject(find.byType(RichText));

    expect(glyphText.text.style!.color, Colors.green);
    expect(glyphText.text.style!.fontSize, 10.0);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(iconTheme: const IconThemeData(color: Colors.orange, size: 20.0)),
        home: const Icon(Icons.computer),
      ),
    );
    await tester.pump(const Duration(milliseconds: 100)); // Halfway through the theme transition

    glyphText = tester.renderObject(find.byType(RichText));

    expect(glyphText.text.style!.color, Color.lerp(Colors.green, Colors.orange, 0.5));
    expect(glyphText.text.style!.fontSize, 15.0);

    await tester.pump(const Duration(milliseconds: 100)); // Finish the transition
    glyphText = tester.renderObject(find.byType(RichText));

    expect(glyphText.text.style!.color, Colors.orange);
    expect(glyphText.text.style!.fontSize, 20.0);
  });

  testWidgets('Same ThemeData reapplied does not trigger descendants rebuilds', (
    WidgetTester tester,
  ) async {
    testBuildCalled = 0;
    ThemeData themeData = ThemeData(primaryColor: const Color(0xFF000000));

    Widget buildTheme() {
      return Theme(data: themeData, child: const Test());
    }

    await tester.pumpWidget(buildTheme());
    expect(testBuildCalled, 1);

    // Pump the same widgets again.
    await tester.pumpWidget(buildTheme());
    // No repeated build calls to the child since it's the same theme data.
    expect(testBuildCalled, 1);

    // New instance of theme data but still the same content.
    themeData = ThemeData(primaryColor: const Color(0xFF000000));
    await tester.pumpWidget(buildTheme());
    // Still no repeated calls.
    expect(testBuildCalled, 1);

    // Different now.
    themeData = ThemeData(primaryColor: const Color(0xFF222222));
    await tester.pumpWidget(buildTheme());
    // Should call build again.
    expect(testBuildCalled, 2);
  });

  testWidgets('Text geometry set in Theme has higher precedence than that of Localizations', (
    WidgetTester tester,
  ) async {
    const double kMagicFontSize = 4321.0;
    final ThemeData fallback = ThemeData.fallback();
    final ThemeData customTheme = fallback.copyWith(
      primaryTextTheme: fallback.primaryTextTheme.copyWith(
        bodyMedium: fallback.primaryTextTheme.bodyMedium!.copyWith(fontSize: kMagicFontSize),
      ),
    );
    expect(customTheme.primaryTextTheme.bodyMedium!.fontSize, kMagicFontSize);

    late double actualFontSize;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Theme(
          data: customTheme,
          child: Builder(
            builder: (BuildContext context) {
              final ThemeData theme = Theme.of(context);
              actualFontSize = theme.primaryTextTheme.bodyMedium!.fontSize!;
              return Text('A', style: theme.primaryTextTheme.bodyMedium);
            },
          ),
        ),
      ),
    );

    expect(actualFontSize, kMagicFontSize);
  });

  testWidgets('Material2 - Default Theme provides all basic TextStyle properties', (
    WidgetTester tester,
  ) async {
    late ThemeData theme;
    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Builder(
            builder: (BuildContext context) {
              theme = Theme.of(context);
              return const Text('A');
            },
          ),
        ),
      ),
    );

    List<TextStyle> extractStyles(TextTheme textTheme) {
      return <TextStyle>[
        textTheme.displayLarge!,
        textTheme.displayMedium!,
        textTheme.displaySmall!,
        textTheme.headlineLarge!,
        textTheme.headlineMedium!,
        textTheme.headlineSmall!,
        textTheme.titleLarge!,
        textTheme.titleMedium!,
        textTheme.bodyLarge!,
        textTheme.bodyMedium!,
        textTheme.bodySmall!,
        textTheme.labelLarge!,
        textTheme.labelMedium!,
        // textTheme.labelSmall!,
      ];
    }

    for (final TextTheme textTheme in <TextTheme>[theme.textTheme, theme.primaryTextTheme]) {
      for (final TextStyle style in extractStyles(
        textTheme,
      ).map<TextStyle>((TextStyle style) => _TextStyleProxy(style))) {
        expect(style.inherit, false);
        expect(style.color, isNotNull);
        expect(style.fontFamily, isNotNull);
        expect(style.fontSize, isNotNull);
        expect(style.fontWeight, isNotNull);
        expect(style.fontStyle, null);
        expect(style.letterSpacing, null);
        expect(style.wordSpacing, null);
        expect(style.textBaseline, isNotNull);
        expect(style.height, null);
        expect(style.decoration, TextDecoration.none);
        expect(style.decorationColor, null);
        expect(style.decorationStyle, null);
        expect(style.debugLabel, isNotNull);
        expect(style.locale, null);
        expect(style.background, null);
      }
    }

    expect(
      theme.textTheme.displayLarge!.debugLabel,
      '(englishLike displayLarge 2014).merge(blackMountainView displayLarge)',
    );
  });

  testWidgets('Material3 - Default Theme provides all basic TextStyle properties', (
    WidgetTester tester,
  ) async {
    late ThemeData theme;
    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: true),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Builder(
            builder: (BuildContext context) {
              theme = Theme.of(context);
              return const Text('A');
            },
          ),
        ),
      ),
    );

    List<TextStyle> extractStyles(TextTheme textTheme) {
      return <TextStyle>[
        textTheme.displayLarge!,
        textTheme.displayMedium!,
        textTheme.displaySmall!,
        textTheme.headlineLarge!,
        textTheme.headlineMedium!,
        textTheme.headlineSmall!,
        textTheme.titleLarge!,
        textTheme.titleMedium!,
        textTheme.bodyLarge!,
        textTheme.bodyMedium!,
        textTheme.bodySmall!,
        textTheme.labelLarge!,
        textTheme.labelMedium!,
      ];
    }

    for (final TextTheme textTheme in <TextTheme>[theme.textTheme, theme.primaryTextTheme]) {
      for (final TextStyle style in extractStyles(
        textTheme,
      ).map<TextStyle>((TextStyle style) => _TextStyleProxy(style))) {
        expect(style.inherit, false);
        expect(style.color, isNotNull);
        expect(style.fontFamily, isNotNull);
        expect(style.fontSize, isNotNull);
        expect(style.fontWeight, isNotNull);
        expect(style.fontStyle, null);
        expect(style.letterSpacing, isNotNull);
        expect(style.wordSpacing, null);
        expect(style.textBaseline, isNotNull);
        expect(style.height, isNotNull);
        expect(style.decoration, TextDecoration.none);
        expect(style.decorationColor, isNotNull);
        expect(style.decorationStyle, null);
        expect(style.debugLabel, isNotNull);
        expect(style.locale, null);
        expect(style.background, null);
      }
    }

    expect(
      theme.textTheme.displayLarge!.debugLabel,
      '(englishLike displayLarge 2021).merge((blackMountainView displayLarge).apply)',
    );
  });

  group('Cupertino theme', () {
    late int buildCount;
    CupertinoThemeData? actualTheme;
    IconThemeData? actualIconTheme;
    BuildContext? context;

    final Widget singletonThemeSubtree = Builder(
      builder: (BuildContext localContext) {
        buildCount++;
        actualTheme = CupertinoTheme.of(localContext);
        actualIconTheme = IconTheme.of(localContext);
        context = localContext;
        return const Placeholder();
      },
    );

    Future<CupertinoThemeData> testTheme(WidgetTester tester, ThemeData theme) async {
      await tester.pumpWidget(Theme(data: theme, child: singletonThemeSubtree));
      return actualTheme!;
    }

    setUp(() {
      buildCount = 0;
      actualTheme = null;
      actualIconTheme = null;
      context = null;
    });

    testWidgets('Material2 - Default light theme has defaults', (WidgetTester tester) async {
      final CupertinoThemeData themeM2 = await testTheme(tester, ThemeData(useMaterial3: false));

      expect(themeM2.brightness, Brightness.light);
      expect(themeM2.primaryColor, Colors.blue);
      expect(themeM2.scaffoldBackgroundColor, Colors.grey[50]);
      expect(themeM2.primaryContrastingColor, Colors.white);
      expect(themeM2.textTheme.textStyle.fontFamily, 'CupertinoSystemText');
      expect(themeM2.textTheme.textStyle.fontSize, 17.0);
    });

    testWidgets('Material3 - Default light theme has defaults', (WidgetTester tester) async {
      final CupertinoThemeData themeM3 = await testTheme(tester, ThemeData(useMaterial3: true));

      expect(themeM3.brightness, Brightness.light);
      expect(themeM3.primaryColor, const Color(0xff6750a4));
      expect(themeM3.scaffoldBackgroundColor, const Color(0xfffef7ff)); // ColorScheme.background
      expect(themeM3.primaryContrastingColor, Colors.white);
      expect(themeM3.textTheme.textStyle.fontFamily, 'CupertinoSystemText');
      expect(themeM3.textTheme.textStyle.fontSize, 17.0);
    });

    testWidgets('Material2 - Dark theme has defaults', (WidgetTester tester) async {
      final CupertinoThemeData themeM2 = await testTheme(
        tester,
        ThemeData.dark(useMaterial3: false),
      );

      expect(themeM2.brightness, Brightness.dark);
      expect(themeM2.primaryColor, Colors.blue);
      expect(themeM2.primaryContrastingColor, Colors.white);
      expect(themeM2.scaffoldBackgroundColor, Colors.grey[850]);
      expect(themeM2.textTheme.textStyle.fontFamily, 'CupertinoSystemText');
      expect(themeM2.textTheme.textStyle.fontSize, 17.0);
    });

    testWidgets('Material3 - Dark theme has defaults', (WidgetTester tester) async {
      final CupertinoThemeData themeM3 = await testTheme(
        tester,
        ThemeData.dark(useMaterial3: true),
      );

      expect(themeM3.brightness, Brightness.dark);
      expect(themeM3.primaryColor, const Color(0xffd0bcff));
      expect(themeM3.primaryContrastingColor, const Color(0xff381e72));
      expect(themeM3.scaffoldBackgroundColor, const Color(0xff141218));
      expect(themeM3.textTheme.textStyle.fontFamily, 'CupertinoSystemText');
      expect(themeM3.textTheme.textStyle.fontSize, 17.0);
    });

    testWidgets('MaterialTheme overrides the brightness', (WidgetTester tester) async {
      await testTheme(tester, ThemeData.dark());
      expect(CupertinoTheme.brightnessOf(context!), Brightness.dark);

      await testTheme(tester, ThemeData.light());
      expect(CupertinoTheme.brightnessOf(context!), Brightness.light);

      // Overridable by cupertinoOverrideTheme.
      await testTheme(
        tester,
        ThemeData(
          brightness: Brightness.light,
          cupertinoOverrideTheme: const CupertinoThemeData(brightness: Brightness.dark),
        ),
      );
      expect(CupertinoTheme.brightnessOf(context!), Brightness.dark);

      await testTheme(
        tester,
        ThemeData(
          brightness: Brightness.dark,
          cupertinoOverrideTheme: const CupertinoThemeData(brightness: Brightness.light),
        ),
      );
      expect(CupertinoTheme.brightnessOf(context!), Brightness.light);
    });

    testWidgets('Material2 - Can override material theme', (WidgetTester tester) async {
      final CupertinoThemeData themeM2 = await testTheme(
        tester,
        ThemeData(
          cupertinoOverrideTheme: const CupertinoThemeData(
            scaffoldBackgroundColor: CupertinoColors.lightBackgroundGray,
          ),
          useMaterial3: false,
        ),
      );

      expect(themeM2.brightness, Brightness.light);
      // We took the scaffold background override but the rest are still cascaded
      // to the material themeM2.
      expect(themeM2.primaryColor, Colors.blue);
      expect(themeM2.primaryContrastingColor, Colors.white);
      expect(themeM2.scaffoldBackgroundColor, CupertinoColors.lightBackgroundGray);
      expect(themeM2.textTheme.textStyle.fontFamily, 'CupertinoSystemText');
      expect(themeM2.textTheme.textStyle.fontSize, 17.0);
    });

    testWidgets('Material3 - Can override material theme', (WidgetTester tester) async {
      final CupertinoThemeData themeM3 = await testTheme(
        tester,
        ThemeData(
          cupertinoOverrideTheme: const CupertinoThemeData(
            scaffoldBackgroundColor: CupertinoColors.lightBackgroundGray,
          ),
          useMaterial3: true,
        ),
      );

      expect(themeM3.brightness, Brightness.light);
      // We took the scaffold background override but the rest are still cascaded
      // to the material themeM3.
      expect(themeM3.primaryColor, const Color(0xff6750a4));
      expect(themeM3.primaryContrastingColor, Colors.white);
      expect(themeM3.scaffoldBackgroundColor, CupertinoColors.lightBackgroundGray);
      expect(themeM3.textTheme.textStyle.fontFamily, 'CupertinoSystemText');
      expect(themeM3.textTheme.textStyle.fontSize, 17.0);
    });

    testWidgets('Material2 - Can override properties that are independent of material', (
      WidgetTester tester,
    ) async {
      final CupertinoThemeData themeM2 = await testTheme(
        tester,
        ThemeData(
          cupertinoOverrideTheme: const CupertinoThemeData(
            // The bar colors ignore all things material except brightness.
            barBackgroundColor: CupertinoColors.black,
          ),
          useMaterial3: false,
        ),
      );

      expect(themeM2.primaryColor, Colors.blue);
      // MaterialBasedCupertinoThemeData should also function like a normal CupertinoThemeData.
      expect(themeM2.barBackgroundColor, CupertinoColors.black);
    });

    testWidgets('Material3 - Can override properties that are independent of material', (
      WidgetTester tester,
    ) async {
      final CupertinoThemeData themeM3 = await testTheme(
        tester,
        ThemeData(
          cupertinoOverrideTheme: const CupertinoThemeData(
            // The bar colors ignore all things material except brightness.
            barBackgroundColor: CupertinoColors.black,
          ),
          useMaterial3: true,
        ),
      );

      expect(themeM3.primaryColor, const Color(0xff6750a4));
      // MaterialBasedCupertinoThemeData should also function like a normal CupertinoThemeData.
      expect(themeM3.barBackgroundColor, CupertinoColors.black);
    });

    testWidgets('Material2 - Changing material theme triggers rebuilds', (
      WidgetTester tester,
    ) async {
      CupertinoThemeData themeM2 = await testTheme(
        tester,
        ThemeData(useMaterial3: false, primarySwatch: Colors.red),
      );

      expect(buildCount, 1);
      expect(themeM2.primaryColor, Colors.red);

      themeM2 = await testTheme(
        tester,
        ThemeData(useMaterial3: false, primarySwatch: Colors.orange),
      );

      expect(buildCount, 2);
      expect(themeM2.primaryColor, Colors.orange);
    });

    testWidgets('Material3 - Changing material theme triggers rebuilds', (
      WidgetTester tester,
    ) async {
      CupertinoThemeData themeM3 = await testTheme(
        tester,
        ThemeData(useMaterial3: true, colorScheme: const ColorScheme.light(primary: Colors.red)),
      );

      expect(buildCount, 1);
      expect(themeM3.primaryColor, Colors.red);

      themeM3 = await testTheme(
        tester,
        ThemeData(useMaterial3: true, colorScheme: const ColorScheme.light(primary: Colors.orange)),
      );

      expect(buildCount, 2);
      expect(themeM3.primaryColor, Colors.orange);
    });

    testWidgets("CupertinoThemeData does not override material theme's icon theme", (
      WidgetTester tester,
    ) async {
      const Color materialIconColor = Colors.blue;
      const Color cupertinoIconColor = Colors.black;

      await testTheme(
        tester,
        ThemeData(
          iconTheme: const IconThemeData(color: materialIconColor),
          cupertinoOverrideTheme: const CupertinoThemeData(primaryColor: cupertinoIconColor),
        ),
      );

      expect(buildCount, 1);
      expect(actualIconTheme!.color, materialIconColor);
    });

    testWidgets('Changing cupertino theme override triggers rebuilds', (WidgetTester tester) async {
      CupertinoThemeData theme = await testTheme(
        tester,
        ThemeData(
          primarySwatch: Colors.purple,
          cupertinoOverrideTheme: const CupertinoThemeData(
            primaryColor: CupertinoColors.activeOrange,
          ),
        ),
      );

      expect(buildCount, 1);
      expect(theme.primaryColor, CupertinoColors.activeOrange);

      theme = await testTheme(
        tester,
        ThemeData(
          primarySwatch: Colors.purple,
          cupertinoOverrideTheme: const CupertinoThemeData(
            primaryColor: CupertinoColors.activeGreen,
          ),
        ),
      );

      expect(buildCount, 2);
      expect(theme.primaryColor, CupertinoColors.activeGreen);
    });

    testWidgets('Cupertino theme override blocks derivative changes', (WidgetTester tester) async {
      CupertinoThemeData theme = await testTheme(
        tester,
        ThemeData(
          primarySwatch: Colors.purple,
          cupertinoOverrideTheme: const CupertinoThemeData(
            primaryColor: CupertinoColors.activeOrange,
          ),
        ),
      );

      expect(buildCount, 1);
      expect(theme.primaryColor, CupertinoColors.activeOrange);

      // Change the upstream material primary color.
      theme = await testTheme(
        tester,
        ThemeData(
          primarySwatch: Colors.blue,
          cupertinoOverrideTheme: const CupertinoThemeData(
            // But the primary material color is preempted by the override.
            primaryColor: CupertinoColors.systemRed,
          ),
        ),
      );

      expect(buildCount, 2);
      expect(theme.primaryColor, CupertinoColors.systemRed);
    });

    testWidgets(
      'Material2 - Cupertino overrides do not block derivatives triggering rebuilds when derivatives are not overridden',
      (WidgetTester tester) async {
        CupertinoThemeData theme = await testTheme(
          tester,
          ThemeData(
            useMaterial3: false,
            primarySwatch: Colors.purple,
            cupertinoOverrideTheme: const CupertinoThemeData(
              primaryContrastingColor: CupertinoColors.destructiveRed,
            ),
          ),
        );

        expect(buildCount, 1);
        expect(theme.textTheme.actionTextStyle.color, Colors.purple);
        expect(theme.primaryContrastingColor, CupertinoColors.destructiveRed);

        theme = await testTheme(
          tester,
          ThemeData(
            useMaterial3: false,
            primarySwatch: Colors.green,
            cupertinoOverrideTheme: const CupertinoThemeData(
              primaryContrastingColor: CupertinoColors.destructiveRed,
            ),
          ),
        );

        expect(buildCount, 2);
        expect(theme.textTheme.actionTextStyle.color, Colors.green);
        expect(theme.primaryContrastingColor, CupertinoColors.destructiveRed);
      },
    );

    testWidgets(
      'Material3 - Cupertino overrides do not block derivatives triggering rebuilds when derivatives are not overridden',
      (WidgetTester tester) async {
        CupertinoThemeData theme = await testTheme(
          tester,
          ThemeData(
            useMaterial3: true,
            colorScheme: const ColorScheme.light(primary: Colors.purple),
            cupertinoOverrideTheme: const CupertinoThemeData(
              primaryContrastingColor: CupertinoColors.destructiveRed,
            ),
          ),
        );

        expect(buildCount, 1);
        expect(theme.textTheme.actionTextStyle.color, Colors.purple);
        expect(theme.primaryContrastingColor, CupertinoColors.destructiveRed);

        theme = await testTheme(
          tester,
          ThemeData(
            useMaterial3: true,
            colorScheme: const ColorScheme.light(primary: Colors.green),
            cupertinoOverrideTheme: const CupertinoThemeData(
              primaryContrastingColor: CupertinoColors.destructiveRed,
            ),
          ),
        );

        expect(buildCount, 2);
        expect(theme.textTheme.actionTextStyle.color, Colors.green);
        expect(theme.primaryContrastingColor, CupertinoColors.destructiveRed);
      },
    );

    testWidgets(
      'Material2 - copyWith only copies the overrides, not the material or cupertino derivatives',
      (WidgetTester tester) async {
        final CupertinoThemeData originalTheme = await testTheme(
          tester,
          ThemeData(
            useMaterial3: false,
            primarySwatch: Colors.purple,
            cupertinoOverrideTheme: const CupertinoThemeData(
              primaryContrastingColor: CupertinoColors.activeOrange,
            ),
          ),
        );

        final CupertinoThemeData copiedTheme = originalTheme.copyWith(
          barBackgroundColor: CupertinoColors.destructiveRed,
        );

        final CupertinoThemeData theme = await testTheme(
          tester,
          ThemeData(
            useMaterial3: false,
            primarySwatch: Colors.blue,
            cupertinoOverrideTheme: copiedTheme,
          ),
        );

        expect(theme.primaryColor, Colors.blue);
        expect(theme.primaryContrastingColor, CupertinoColors.activeOrange);
        expect(theme.barBackgroundColor, CupertinoColors.destructiveRed);
      },
    );

    testWidgets(
      'Material3 - copyWith only copies the overrides, not the material or cupertino derivatives',
      (WidgetTester tester) async {
        final CupertinoThemeData originalTheme = await testTheme(
          tester,
          ThemeData(
            useMaterial3: true,
            colorScheme: const ColorScheme.light(primary: Colors.purple),
            cupertinoOverrideTheme: const CupertinoThemeData(
              primaryContrastingColor: CupertinoColors.activeOrange,
            ),
          ),
        );

        final CupertinoThemeData copiedTheme = originalTheme.copyWith(
          barBackgroundColor: CupertinoColors.destructiveRed,
        );

        final CupertinoThemeData theme = await testTheme(
          tester,
          ThemeData(
            useMaterial3: true,
            colorScheme: const ColorScheme.light(primary: Colors.blue),
            cupertinoOverrideTheme: copiedTheme,
          ),
        );

        expect(theme.primaryColor, Colors.blue);
        expect(theme.primaryContrastingColor, CupertinoColors.activeOrange);
        expect(theme.barBackgroundColor, CupertinoColors.destructiveRed);
      },
    );

    testWidgets("Material2 - Material themes with no cupertino overrides can also be copyWith'ed", (
      WidgetTester tester,
    ) async {
      final CupertinoThemeData originalTheme = await testTheme(
        tester,
        ThemeData(useMaterial3: false, primarySwatch: Colors.purple),
      );

      final CupertinoThemeData copiedTheme = originalTheme.copyWith(
        primaryContrastingColor: CupertinoColors.destructiveRed,
      );

      final CupertinoThemeData theme = await testTheme(
        tester,
        ThemeData(
          useMaterial3: false,
          primarySwatch: Colors.blue,
          cupertinoOverrideTheme: copiedTheme,
        ),
      );

      expect(theme.primaryColor, Colors.blue);
      expect(theme.primaryContrastingColor, CupertinoColors.destructiveRed);
    });

    testWidgets("Material3 - Material themes with no cupertino overrides can also be copyWith'ed", (
      WidgetTester tester,
    ) async {
      final CupertinoThemeData originalTheme = await testTheme(
        tester,
        ThemeData(useMaterial3: true, colorScheme: const ColorScheme.light(primary: Colors.purple)),
      );

      final CupertinoThemeData copiedTheme = originalTheme.copyWith(
        primaryContrastingColor: CupertinoColors.destructiveRed,
      );

      final CupertinoThemeData theme = await testTheme(
        tester,
        ThemeData(
          useMaterial3: true,
          colorScheme: const ColorScheme.light(primary: Colors.blue),
          cupertinoOverrideTheme: copiedTheme,
        ),
      );

      expect(theme.primaryColor, Colors.blue);
      expect(theme.primaryContrastingColor, CupertinoColors.destructiveRed);
    });
  });
}

int testBuildCalled = 0;

class Test extends StatelessWidget {
  const Test({super.key});

  @override
  Widget build(BuildContext context) {
    testBuildCalled += 1;
    return Container(decoration: BoxDecoration(color: Theme.of(context).primaryColor));
  }
}

/// This class exists only to make sure that we test all the properties of the
/// [TextStyle] class. If a property is added/removed/renamed, the analyzer will
/// complain that this class has incorrect overrides.
class _TextStyleProxy implements TextStyle {
  _TextStyleProxy(this._delegate);

  final TextStyle _delegate;

  // Do make sure that all the properties correctly forward to the _delegate.
  @override
  Color? get color => _delegate.color;
  @override
  Color? get backgroundColor => _delegate.backgroundColor;
  @override
  String? get debugLabel => _delegate.debugLabel;
  @override
  TextDecoration? get decoration => _delegate.decoration;
  @override
  Color? get decorationColor => _delegate.decorationColor;
  @override
  TextDecorationStyle? get decorationStyle => _delegate.decorationStyle;
  @override
  double? get decorationThickness => _delegate.decorationThickness;
  @override
  String? get fontFamily => _delegate.fontFamily;
  @override
  List<String>? get fontFamilyFallback => _delegate.fontFamilyFallback;
  @override
  double? get fontSize => _delegate.fontSize;
  @override
  FontStyle? get fontStyle => _delegate.fontStyle;
  @override
  FontWeight? get fontWeight => _delegate.fontWeight;
  @override
  double? get height => _delegate.height;
  @override
  TextLeadingDistribution? get leadingDistribution => _delegate.leadingDistribution;
  @override
  Locale? get locale => _delegate.locale;
  @override
  ui.Paint? get foreground => _delegate.foreground;
  @override
  ui.Paint? get background => _delegate.background;
  @override
  bool get inherit => _delegate.inherit;
  @override
  double? get letterSpacing => _delegate.letterSpacing;
  @override
  TextBaseline? get textBaseline => _delegate.textBaseline;
  @override
  double? get wordSpacing => _delegate.wordSpacing;
  @override
  List<Shadow>? get shadows => _delegate.shadows;
  @override
  List<ui.FontFeature>? get fontFeatures => _delegate.fontFeatures;
  @override
  List<ui.FontVariation>? get fontVariations => _delegate.fontVariations;
  @override
  TextOverflow? get overflow => _delegate.overflow;

  @override
  String toString({DiagnosticLevel minLevel = DiagnosticLevel.info}) => super.toString();

  @override
  DiagnosticsNode toDiagnosticsNode({String? name, DiagnosticsTreeStyle? style}) {
    throw UnimplementedError();
  }

  @override
  String toStringShort() {
    throw UnimplementedError();
  }

  @override
  TextStyle apply({
    Color? color,
    Color? backgroundColor,
    TextDecoration? decoration,
    Color? decorationColor,
    TextDecorationStyle? decorationStyle,
    double decorationThicknessFactor = 1.0,
    double decorationThicknessDelta = 0.0,
    String? fontFamily,
    List<String>? fontFamilyFallback,
    double fontSizeFactor = 1.0,
    double fontSizeDelta = 0.0,
    int fontWeightDelta = 0,
    FontStyle? fontStyle,
    double letterSpacingFactor = 1.0,
    double letterSpacingDelta = 0.0,
    double wordSpacingFactor = 1.0,
    double wordSpacingDelta = 0.0,
    double heightFactor = 1.0,
    double heightDelta = 0.0,
    TextLeadingDistribution? leadingDistribution,
    TextBaseline? textBaseline,
    Locale? locale,
    List<ui.Shadow>? shadows,
    List<ui.FontFeature>? fontFeatures,
    List<ui.FontVariation>? fontVariations,
    TextOverflow? overflow,
    String? package,
  }) {
    throw UnimplementedError();
  }

  @override
  RenderComparison compareTo(TextStyle other) {
    throw UnimplementedError();
  }

  @override
  TextStyle copyWith({
    bool? inherit,
    Color? color,
    Color? backgroundColor,
    String? fontFamily,
    List<String>? fontFamilyFallback,
    double? fontSize,
    FontWeight? fontWeight,
    FontStyle? fontStyle,
    double? letterSpacing,
    double? wordSpacing,
    TextBaseline? textBaseline,
    double? height,
    TextLeadingDistribution? leadingDistribution,
    Locale? locale,
    ui.Paint? foreground,
    ui.Paint? background,
    List<Shadow>? shadows,
    List<ui.FontFeature>? fontFeatures,
    List<ui.FontVariation>? fontVariations,
    TextDecoration? decoration,
    Color? decorationColor,
    TextDecorationStyle? decorationStyle,
    double? decorationThickness,
    String? debugLabel,
    TextOverflow? overflow,
    String? package,
  }) {
    throw UnimplementedError();
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties, {String prefix = ''}) {
    throw UnimplementedError();
  }

  @override
  ui.ParagraphStyle getParagraphStyle({
    TextAlign? textAlign,
    TextDirection? textDirection,
    double textScaleFactor = 1.0,
    TextScaler textScaler = TextScaler.noScaling,
    String? ellipsis,
    int? maxLines,
    ui.TextHeightBehavior? textHeightBehavior,
    Locale? locale,
    String? fontFamily,
    double? fontSize,
    FontWeight? fontWeight,
    FontStyle? fontStyle,
    double? height,
    StrutStyle? strutStyle,
  }) {
    throw UnimplementedError();
  }

  @override
  ui.TextStyle getTextStyle({
    double textScaleFactor = 1.0,
    TextScaler textScaler = TextScaler.noScaling,
  }) {
    throw UnimplementedError();
  }

  @override
  TextStyle merge(TextStyle? other) {
    throw UnimplementedError();
  }
}
