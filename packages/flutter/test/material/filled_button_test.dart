// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';
import '../widgets/semantics_tester.dart';

void main() {
  TextStyle iconStyle(WidgetTester tester, IconData icon) {
    final RichText iconRichText = tester.widget<RichText>(
      find.descendant(of: find.byIcon(icon), matching: find.byType(RichText)),
    );
    return iconRichText.text.style!;
  }

  testWidgets('FilledButton, FilledButton.icon defaults', (WidgetTester tester) async {
    const ColorScheme colorScheme = ColorScheme.light();
    final ThemeData theme = ThemeData.from(useMaterial3: false, colorScheme: colorScheme);

    // Enabled FilledButton
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Center(child: FilledButton(onPressed: () {}, child: const Text('button'))),
      ),
    );

    final Finder buttonMaterial = find.descendant(
      of: find.byType(FilledButton),
      matching: find.byType(Material),
    );

    Material material = tester.widget<Material>(buttonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.primary);
    expect(material.elevation, 0);
    expect(material.shadowColor, const Color(0xff000000));
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle!.color, colorScheme.onPrimary);
    expect(material.textStyle!.fontFamily, 'Roboto');
    expect(material.textStyle!.fontSize, 14);
    expect(material.textStyle!.fontWeight, FontWeight.w500);
    expect(material.type, MaterialType.button);

    final Align align = tester.firstWidget<Align>(
      find.ancestor(of: find.text('button'), matching: find.byType(Align)),
    );
    expect(align.alignment, Alignment.center);

    final Offset center = tester.getCenter(find.byType(FilledButton));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pump(); // start the splash animation
    await tester.pump(const Duration(milliseconds: 100)); // splash is underway

    // Enabled FilledButton.icon
    final Key iconButtonKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Center(
          child: FilledButton.icon(
            key: iconButtonKey,
            onPressed: () {},
            icon: const Icon(Icons.add),
            label: const Text('label'),
          ),
        ),
      ),
    );

    final Finder iconButtonMaterial = find.descendant(
      of: find.byKey(iconButtonKey),
      matching: find.byType(Material),
    );

    material = tester.widget<Material>(iconButtonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.primary);
    expect(material.elevation, 0);
    expect(material.shadowColor, const Color(0xff000000));
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle!.color, colorScheme.onPrimary);
    expect(material.textStyle!.fontFamily, 'Roboto');
    expect(material.textStyle!.fontSize, 14);
    expect(material.textStyle!.fontWeight, FontWeight.w500);
    expect(material.type, MaterialType.button);

    // Disabled FilledButton
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Center(child: FilledButton(onPressed: null, child: Text('button'))),
      ),
    );

    // Finish the elevation animation, final background color change.
    await tester.pumpAndSettle();

    material = tester.widget<Material>(buttonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.onSurface.withOpacity(0.12));
    expect(material.elevation, 0.0);
    expect(material.shadowColor, const Color(0xff000000));
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle!.color, colorScheme.onSurface.withOpacity(0.38));
    expect(material.textStyle!.fontFamily, 'Roboto');
    expect(material.textStyle!.fontSize, 14);
    expect(material.textStyle!.fontWeight, FontWeight.w500);
    expect(material.type, MaterialType.button);

    // Finish gesture to release resources.
    await gesture.up();
    await tester.pumpAndSettle();
  });

  testWidgets('FilledButton.defaultStyle produces a ButtonStyle with appropriate non-null values', (
    WidgetTester tester,
  ) async {
    const ColorScheme colorScheme = ColorScheme.light();
    final ThemeData theme = ThemeData.from(colorScheme: colorScheme);

    final FilledButton button = FilledButton(onPressed: () {}, child: const Text('button'));
    BuildContext? capturedContext;
    // Enabled FilledButton
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Center(
          child: Builder(
            builder: (BuildContext context) {
              capturedContext = context;
              return button;
            },
          ),
        ),
      ),
    );
    final ButtonStyle style = button.defaultStyleOf(capturedContext!);

    // Properties that must be non-null.
    expect(style.textStyle, isNotNull, reason: 'textStyle style');
    expect(style.backgroundColor, isNotNull, reason: 'backgroundColor style');
    expect(style.foregroundColor, isNotNull, reason: 'foregroundColor style');
    expect(style.overlayColor, isNotNull, reason: 'overlayColor style');
    expect(style.shadowColor, isNotNull, reason: 'shadowColor style');
    expect(style.surfaceTintColor, isNotNull, reason: 'surfaceTintColor style');
    expect(style.elevation, isNotNull, reason: 'elevation style');
    expect(style.padding, isNotNull, reason: 'padding style');
    expect(style.minimumSize, isNotNull, reason: 'minimumSize style');
    expect(style.maximumSize, isNotNull, reason: 'maximumSize style');
    expect(style.iconColor, isNotNull, reason: 'iconColor style');
    expect(style.iconSize, isNotNull, reason: 'iconSize style');
    expect(style.shape, isNotNull, reason: 'shape style');
    expect(style.mouseCursor, isNotNull, reason: 'mouseCursor style');
    expect(style.visualDensity, isNotNull, reason: 'visualDensity style');
    expect(style.tapTargetSize, isNotNull, reason: 'tapTargetSize style');
    expect(style.animationDuration, isNotNull, reason: 'animationDuration style');
    expect(style.enableFeedback, isNotNull, reason: 'enableFeedback style');
    expect(style.alignment, isNotNull, reason: 'alignment style');
    expect(style.splashFactory, isNotNull, reason: 'splashFactory style');

    // Properties that are expected to be null.
    expect(style.fixedSize, isNull, reason: 'fixedSize style');
    expect(style.side, isNull, reason: 'side style');
    expect(style.backgroundBuilder, isNull, reason: 'backgroundBuilder style');
    expect(style.foregroundBuilder, isNull, reason: 'foregroundBuilder style');
  });

  testWidgets(
    'FilledButton.defaultStyle with an icon produces a ButtonStyle with appropriate non-null values',
    (WidgetTester tester) async {
      const ColorScheme colorScheme = ColorScheme.light();
      final ThemeData theme = ThemeData.from(colorScheme: colorScheme);

      final FilledButton button = FilledButton.icon(
        onPressed: () {},
        icon: const SizedBox(),
        label: const Text('button'),
      );
      BuildContext? capturedContext;
      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: Center(
            child: Builder(
              builder: (BuildContext context) {
                capturedContext = context;
                return button;
              },
            ),
          ),
        ),
      );
      final ButtonStyle style = button.defaultStyleOf(capturedContext!);

      // Properties that must be non-null.
      expect(style.textStyle, isNotNull, reason: 'textStyle style');
      expect(style.backgroundColor, isNotNull, reason: 'backgroundColor style');
      expect(style.foregroundColor, isNotNull, reason: 'foregroundColor style');
      expect(style.overlayColor, isNotNull, reason: 'overlayColor style');
      expect(style.shadowColor, isNotNull, reason: 'shadowColor style');
      expect(style.surfaceTintColor, isNotNull, reason: 'surfaceTintColor style');
      expect(style.elevation, isNotNull, reason: 'elevation style');
      expect(style.padding, isNotNull, reason: 'padding style');
      expect(style.minimumSize, isNotNull, reason: 'minimumSize style');
      expect(style.maximumSize, isNotNull, reason: 'maximumSize style');
      expect(style.iconColor, isNotNull, reason: 'iconColor style');
      expect(style.iconSize, isNotNull, reason: 'iconSize style');
      expect(style.shape, isNotNull, reason: 'shape style');
      expect(style.mouseCursor, isNotNull, reason: 'mouseCursor style');
      expect(style.visualDensity, isNotNull, reason: 'visualDensity style');
      expect(style.tapTargetSize, isNotNull, reason: 'tapTargetSize style');
      expect(style.animationDuration, isNotNull, reason: 'animationDuration style');
      expect(style.enableFeedback, isNotNull, reason: 'enableFeedback style');
      expect(style.alignment, isNotNull, reason: 'alignment style');
      expect(style.splashFactory, isNotNull, reason: 'splashFactory style');

      // Properties that are expected to be null.
      expect(style.fixedSize, isNull, reason: 'fixedSize style');
      expect(style.side, isNull, reason: 'side style');
      expect(style.backgroundBuilder, isNull, reason: 'backgroundBuilder style');
      expect(style.foregroundBuilder, isNull, reason: 'foregroundBuilder style');
    },
  );

  testWidgets('FilledButton.icon produces the correct widgets if icon is null', (
    WidgetTester tester,
  ) async {
    const ColorScheme colorScheme = ColorScheme.light();
    final ThemeData theme = ThemeData.from(colorScheme: colorScheme);
    final Key iconButtonKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Center(
          child: FilledButton.icon(
            key: iconButtonKey,
            onPressed: () {},
            icon: const Icon(Icons.add),
            label: const Text('label'),
          ),
        ),
      ),
    );

    expect(find.byIcon(Icons.add), findsOneWidget);
    expect(find.text('label'), findsOneWidget);

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Center(
          child: FilledButton.icon(
            key: iconButtonKey,
            onPressed: () {},
            // No icon specified.
            label: const Text('label'),
          ),
        ),
      ),
    );

    expect(find.byIcon(Icons.add), findsNothing);
    expect(find.text('label'), findsOneWidget);
  });

  testWidgets('FilledButton.tonalIcon produces the correct widgets if icon is null', (
    WidgetTester tester,
  ) async {
    const ColorScheme colorScheme = ColorScheme.light();
    final ThemeData theme = ThemeData.from(colorScheme: colorScheme);
    final Key iconButtonKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Center(
          child: FilledButton.tonalIcon(
            key: iconButtonKey,
            onPressed: () {},
            icon: const Icon(Icons.add),
            label: const Text('label'),
          ),
        ),
      ),
    );

    expect(find.byIcon(Icons.add), findsOneWidget);
    expect(find.text('label'), findsOneWidget);

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Center(
          child: FilledButton.tonalIcon(
            key: iconButtonKey,
            onPressed: () {},
            // No icon specified.
            label: const Text('label'),
          ),
        ),
      ),
    );

    expect(find.byIcon(Icons.add), findsNothing);
    expect(find.text('label'), findsOneWidget);
  });

  testWidgets('FilledButton.tonal, FilledButton.tonalIcon defaults', (WidgetTester tester) async {
    const ColorScheme colorScheme = ColorScheme.light();
    final ThemeData theme = ThemeData.from(colorScheme: colorScheme);

    // Enabled FilledButton
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Center(child: FilledButton.tonal(onPressed: () {}, child: const Text('button'))),
      ),
    );

    final Finder buttonMaterial = find.descendant(
      of: find.byType(FilledButton),
      matching: find.byType(Material),
    );

    Material material = tester.widget<Material>(buttonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.secondaryContainer);
    expect(material.elevation, 0);
    expect(material.shadowColor, const Color(0xff000000));
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle!.color, colorScheme.onSecondaryContainer);
    expect(material.textStyle!.fontFamily, 'Roboto');
    expect(material.textStyle!.fontSize, 14);
    expect(material.textStyle!.fontWeight, FontWeight.w500);
    expect(material.type, MaterialType.button);

    final Align align = tester.firstWidget<Align>(
      find.ancestor(of: find.text('button'), matching: find.byType(Align)),
    );
    expect(align.alignment, Alignment.center);

    final Offset center = tester.getCenter(find.byType(FilledButton));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pump(); // start the splash animation
    await tester.pump(const Duration(milliseconds: 100)); // splash is underway

    // Enabled FilledButton.tonalIcon
    final Key iconButtonKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Center(
          child: FilledButton.tonalIcon(
            key: iconButtonKey,
            onPressed: () {},
            icon: const Icon(Icons.add),
            label: const Text('label'),
          ),
        ),
      ),
    );

    final Finder iconButtonMaterial = find.descendant(
      of: find.byKey(iconButtonKey),
      matching: find.byType(Material),
    );

    material = tester.widget<Material>(iconButtonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.secondaryContainer);
    expect(material.elevation, 0);
    expect(material.shadowColor, const Color(0xff000000));
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle!.color, colorScheme.onSecondaryContainer);
    expect(material.textStyle!.fontFamily, 'Roboto');
    expect(material.textStyle!.fontSize, 14);
    expect(material.textStyle!.fontWeight, FontWeight.w500);
    expect(material.type, MaterialType.button);

    // Disabled FilledButton
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Center(child: FilledButton.tonal(onPressed: null, child: Text('button'))),
      ),
    );

    // Finish the elevation animation, final background color change.
    await tester.pumpAndSettle();

    material = tester.widget<Material>(buttonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.onSurface.withOpacity(0.12));
    expect(material.elevation, 0.0);
    expect(material.shadowColor, const Color(0xff000000));
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle!.color, colorScheme.onSurface.withOpacity(0.38));
    expect(material.textStyle!.fontFamily, 'Roboto');
    expect(material.textStyle!.fontSize, 14);
    expect(material.textStyle!.fontWeight, FontWeight.w500);
    expect(material.type, MaterialType.button);

    // Finish gesture to release resources.
    await gesture.up();
    await tester.pumpAndSettle();
  });

  testWidgets(
    'Default FilledButton meets a11y contrast guidelines',
    (WidgetTester tester) async {
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData.from(colorScheme: const ColorScheme.light()),
          home: Scaffold(
            body: Center(
              child: FilledButton(
                onPressed: () {},
                focusNode: focusNode,
                child: const Text('FilledButton'),
              ),
            ),
          ),
        ),
      );

      // Default, not disabled.
      await expectLater(tester, meetsGuideline(textContrastGuideline));

      // Focused.
      focusNode.requestFocus();
      await tester.pumpAndSettle();
      await expectLater(tester, meetsGuideline(textContrastGuideline));

      // Hovered.
      final Offset center = tester.getCenter(find.byType(FilledButton));
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      await gesture.moveTo(center);
      await tester.pumpAndSettle();
      await expectLater(tester, meetsGuideline(textContrastGuideline));
    },
    skip: isBrowser, // https://github.com/flutter/flutter/issues/44115
  );

  testWidgets('FilledButton default overlayColor and elevation resolve pressed state', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode();
    final ThemeData theme = ThemeData();

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Center(
            child: Builder(
              builder: (BuildContext context) {
                return FilledButton(
                  onPressed: () {},
                  focusNode: focusNode,
                  child: const Text('FilledButton'),
                );
              },
            ),
          ),
        ),
      ),
    );

    RenderObject overlayColor() {
      return tester.allRenderObjects.firstWhere(
        (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
      );
    }

    double elevation() {
      return tester
          .widget<PhysicalShape>(
            find.descendant(of: find.byType(FilledButton), matching: find.byType(PhysicalShape)),
          )
          .elevation;
    }

    // Hovered.
    final Offset center = tester.getCenter(find.byType(FilledButton));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(elevation(), 1.0);
    expect(overlayColor(), paints..rect(color: theme.colorScheme.onPrimary.withOpacity(0.08)));

    // Highlighted (pressed).
    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(elevation(), 0.0);
    expect(
      overlayColor(),
      paints
        ..rect()
        ..rect(color: theme.colorScheme.onPrimary.withOpacity(0.1)),
    );
    // Remove pressed and hovered states
    await gesture.up();
    await tester.pumpAndSettle();
    await gesture.moveTo(const Offset(0, 50));
    await tester.pumpAndSettle();

    // Focused.
    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(elevation(), 0.0);
    expect(overlayColor(), paints..rect(color: theme.colorScheme.onPrimary.withOpacity(0.1)));
    focusNode.dispose();
  });

  testWidgets('FilledButton.tonal default overlayColor and elevation resolve pressed state', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode();
    final ThemeData theme = ThemeData();

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Center(
            child: Builder(
              builder: (BuildContext context) {
                return FilledButton.tonal(
                  onPressed: () {},
                  focusNode: focusNode,
                  child: const Text('FilledButton'),
                );
              },
            ),
          ),
        ),
      ),
    );

    RenderObject overlayColor() {
      return tester.allRenderObjects.firstWhere(
        (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
      );
    }

    double elevation() {
      return tester
          .widget<PhysicalShape>(
            find.descendant(of: find.byType(FilledButton), matching: find.byType(PhysicalShape)),
          )
          .elevation;
    }

    // Hovered.
    final Offset center = tester.getCenter(find.byType(FilledButton));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(elevation(), 1.0);
    expect(
      overlayColor(),
      paints..rect(color: theme.colorScheme.onSecondaryContainer.withOpacity(0.08)),
    );

    // Highlighted (pressed).
    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(elevation(), 0.0);
    expect(
      overlayColor(),
      paints
        ..rect()
        ..rect(color: theme.colorScheme.onSecondaryContainer.withOpacity(0.1)),
    );
    // Remove pressed and hovered states
    await gesture.up();
    await tester.pumpAndSettle();
    await gesture.moveTo(const Offset(0, 50));
    await tester.pumpAndSettle();

    // Focused.
    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(elevation(), 0.0);
    expect(
      overlayColor(),
      paints..rect(color: theme.colorScheme.onSecondaryContainer.withOpacity(0.1)),
    );
    focusNode.dispose();
  });

  testWidgets('FilledButton uses stateful color for text color in different states', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode();

    const Color pressedColor = Color(0x00000001);
    const Color hoverColor = Color(0x00000002);
    const Color focusedColor = Color(0x00000003);
    const Color defaultColor = Color(0x00000004);

    Color getTextColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.pressed)) {
        return pressedColor;
      }
      if (states.contains(MaterialState.hovered)) {
        return hoverColor;
      }
      if (states.contains(MaterialState.focused)) {
        return focusedColor;
      }
      return defaultColor;
    }

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: FilledButtonTheme(
              data: FilledButtonThemeData(
                style: ButtonStyle(
                  foregroundColor: MaterialStateProperty.resolveWith<Color>(getTextColor),
                ),
              ),
              child: Builder(
                builder: (BuildContext context) {
                  return FilledButton(
                    onPressed: () {},
                    focusNode: focusNode,
                    child: const Text('FilledButton'),
                  );
                },
              ),
            ),
          ),
        ),
      ),
    );

    Color textColor() {
      return tester.renderObject<RenderParagraph>(find.text('FilledButton')).text.style!.color!;
    }

    // Default, not disabled.
    expect(textColor(), equals(defaultColor));

    // Focused.
    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(textColor(), focusedColor);

    // Hovered.
    final Offset center = tester.getCenter(find.byType(FilledButton));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(textColor(), hoverColor);

    // Highlighted (pressed).
    await gesture.down(center);
    await tester.pump(); // Start the splash and highlight animations.
    await tester.pump(
      const Duration(milliseconds: 800),
    ); // Wait for splash and highlight to be well under way.
    expect(textColor(), pressedColor);
    focusNode.dispose();
  });

  testWidgets('FilledButton uses stateful color for icon color in different states', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode();
    final Key buttonKey = UniqueKey();

    const Color pressedColor = Color(0x00000001);
    const Color hoverColor = Color(0x00000002);
    const Color focusedColor = Color(0x00000003);
    const Color defaultColor = Color(0x00000004);

    Color getTextColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.pressed)) {
        return pressedColor;
      }
      if (states.contains(MaterialState.hovered)) {
        return hoverColor;
      }
      if (states.contains(MaterialState.focused)) {
        return focusedColor;
      }
      return defaultColor;
    }

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: FilledButtonTheme(
              data: FilledButtonThemeData(
                style: ButtonStyle(
                  foregroundColor: MaterialStateProperty.resolveWith<Color>(getTextColor),
                  iconColor: MaterialStateProperty.resolveWith<Color>(getTextColor),
                ),
              ),
              child: Builder(
                builder: (BuildContext context) {
                  return FilledButton.icon(
                    key: buttonKey,
                    icon: const Icon(Icons.add),
                    label: const Text('FilledButton'),
                    onPressed: () {},
                    focusNode: focusNode,
                  );
                },
              ),
            ),
          ),
        ),
      ),
    );

    // Default, not disabled.
    expect(iconStyle(tester, Icons.add).color, equals(defaultColor));

    // Focused.
    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(iconStyle(tester, Icons.add).color, focusedColor);

    // Hovered.
    final Offset center = tester.getCenter(find.byKey(buttonKey));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(iconStyle(tester, Icons.add).color, hoverColor);

    // Highlighted (pressed).
    await gesture.down(center);
    await tester.pump(); // Start the splash and highlight animations.
    await tester.pump(
      const Duration(milliseconds: 800),
    ); // Wait for splash and highlight to be well under way.
    expect(iconStyle(tester, Icons.add).color, pressedColor);
    focusNode.dispose();
  });

  testWidgets(
    'FilledButton onPressed and onLongPress callbacks are correctly called when non-null',
    (WidgetTester tester) async {
      bool wasPressed;
      Finder filledButton;

      Widget buildFrame({VoidCallback? onPressed, VoidCallback? onLongPress}) {
        return Directionality(
          textDirection: TextDirection.ltr,
          child: FilledButton(
            onPressed: onPressed,
            onLongPress: onLongPress,
            child: const Text('button'),
          ),
        );
      }

      // onPressed not null, onLongPress null.
      wasPressed = false;
      await tester.pumpWidget(
        buildFrame(
          onPressed: () {
            wasPressed = true;
          },
        ),
      );
      filledButton = find.byType(FilledButton);
      expect(tester.widget<FilledButton>(filledButton).enabled, true);
      await tester.tap(filledButton);
      expect(wasPressed, true);

      // onPressed null, onLongPress not null.
      wasPressed = false;
      await tester.pumpWidget(
        buildFrame(
          onLongPress: () {
            wasPressed = true;
          },
        ),
      );
      filledButton = find.byType(FilledButton);
      expect(tester.widget<FilledButton>(filledButton).enabled, true);
      await tester.longPress(filledButton);
      expect(wasPressed, true);

      // onPressed null, onLongPress null.
      await tester.pumpWidget(buildFrame());
      filledButton = find.byType(FilledButton);
      expect(tester.widget<FilledButton>(filledButton).enabled, false);
    },
  );

  testWidgets('FilledButton onPressed and onLongPress callbacks are distinctly recognized', (
    WidgetTester tester,
  ) async {
    bool didPressButton = false;
    bool didLongPressButton = false;

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: FilledButton(
          onPressed: () {
            didPressButton = true;
          },
          onLongPress: () {
            didLongPressButton = true;
          },
          child: const Text('button'),
        ),
      ),
    );

    final Finder filledButton = find.byType(FilledButton);
    expect(tester.widget<FilledButton>(filledButton).enabled, true);

    expect(didPressButton, isFalse);
    await tester.tap(filledButton);
    expect(didPressButton, isTrue);

    expect(didLongPressButton, isFalse);
    await tester.longPress(filledButton);
    expect(didLongPressButton, isTrue);
  });

  testWidgets("FilledButton response doesn't hover when disabled", (WidgetTester tester) async {
    FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTouch;
    final FocusNode focusNode = FocusNode(debugLabel: 'FilledButton Focus');
    final GlobalKey childKey = GlobalKey();
    bool hovering = false;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SizedBox(
          width: 100,
          height: 100,
          child: FilledButton(
            autofocus: true,
            onPressed: () {},
            onLongPress: () {},
            onHover: (bool value) {
              hovering = value;
            },
            focusNode: focusNode,
            child: SizedBox(key: childKey),
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isTrue);
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byKey(childKey)));
    await tester.pumpAndSettle();
    expect(hovering, isTrue);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: SizedBox(
          width: 100,
          height: 100,
          child: FilledButton(
            focusNode: focusNode,
            onHover: (bool value) {
              hovering = value;
            },
            onPressed: null,
            child: SizedBox(key: childKey),
          ),
        ),
      ),
    );

    await tester.pumpAndSettle();
    expect(focusNode.hasPrimaryFocus, isFalse);
    focusNode.dispose();
  });

  testWidgets('disabled and hovered FilledButton responds to mouse-exit', (
    WidgetTester tester,
  ) async {
    int onHoverCount = 0;
    late bool hover;

    Widget buildFrame({required bool enabled}) {
      return Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: SizedBox(
            width: 100,
            height: 100,
            child: FilledButton(
              onPressed: enabled ? () {} : null,
              onHover: (bool value) {
                onHoverCount += 1;
                hover = value;
              },
              child: const Text('FilledButton'),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(enabled: true));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();

    await gesture.moveTo(tester.getCenter(find.byType(FilledButton)));
    await tester.pumpAndSettle();
    expect(onHoverCount, 1);
    expect(hover, true);

    await tester.pumpWidget(buildFrame(enabled: false));
    await tester.pumpAndSettle();
    await gesture.moveTo(Offset.zero);
    // Even though the FilledButton has been disabled, the mouse-exit still
    // causes onHover(false) to be called.
    expect(onHoverCount, 2);
    expect(hover, false);

    await gesture.moveTo(tester.getCenter(find.byType(FilledButton)));
    await tester.pumpAndSettle();
    // We no longer see hover events because the FilledButton is disabled
    // and it's no longer in the "hovering" state.
    expect(onHoverCount, 2);
    expect(hover, false);

    await tester.pumpWidget(buildFrame(enabled: true));
    await tester.pumpAndSettle();
    // The FilledButton was enabled while it contained the mouse, however
    // we do not call onHover() because it may call setState().
    expect(onHoverCount, 2);
    expect(hover, false);

    await gesture.moveTo(tester.getCenter(find.byType(FilledButton)) - const Offset(1, 1));
    await tester.pumpAndSettle();
    // Moving the mouse a little within the FilledButton doesn't change anything.
    expect(onHoverCount, 2);
    expect(hover, false);
  });

  testWidgets('Can set FilledButton focus and Can set unFocus.', (WidgetTester tester) async {
    final FocusNode node = FocusNode(debugLabel: 'FilledButton Focus');
    bool gotFocus = false;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: FilledButton(
          focusNode: node,
          onFocusChange: (bool focused) => gotFocus = focused,
          onPressed: () {},
          child: const SizedBox(),
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

  testWidgets('When FilledButton disable, Can not set FilledButton focus.', (
    WidgetTester tester,
  ) async {
    final FocusNode node = FocusNode(debugLabel: 'FilledButton Focus');
    bool gotFocus = false;
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: FilledButton(
          focusNode: node,
          onFocusChange: (bool focused) => gotFocus = focused,
          onPressed: null,
          child: const SizedBox(),
        ),
      ),
    );

    node.requestFocus();

    await tester.pump();

    expect(gotFocus, isFalse);
    expect(node.hasFocus, isFalse);
    node.dispose();
  });

  testWidgets('Does FilledButton work with hover', (WidgetTester tester) async {
    const Color hoverColor = Color(0xff001122);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: FilledButton(
          style: ButtonStyle(
            overlayColor: MaterialStateProperty.resolveWith<Color?>((Set<MaterialState> states) {
              return states.contains(MaterialState.hovered) ? hoverColor : null;
            }),
          ),
          onPressed: () {},
          child: const Text('button'),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(find.byType(FilledButton)));
    await tester.pumpAndSettle();

    final RenderObject inkFeatures = tester.allRenderObjects.firstWhere(
      (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
    );
    expect(inkFeatures, paints..rect(color: hoverColor));
  });

  testWidgets('Does FilledButton work with focus', (WidgetTester tester) async {
    const Color focusColor = Color(0xff001122);

    final FocusNode focusNode = FocusNode(debugLabel: 'FilledButton Node');
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: FilledButton(
          style: ButtonStyle(
            overlayColor: MaterialStateProperty.resolveWith<Color?>((Set<MaterialState> states) {
              return states.contains(MaterialState.focused) ? focusColor : null;
            }),
          ),
          focusNode: focusNode,
          onPressed: () {},
          child: const Text('button'),
        ),
      ),
    );

    FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    focusNode.requestFocus();
    await tester.pumpAndSettle();

    final RenderObject inkFeatures = tester.allRenderObjects.firstWhere(
      (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
    );
    expect(inkFeatures, paints..rect(color: focusColor));
    focusNode.dispose();
  });

  testWidgets('Does FilledButton work with autofocus', (WidgetTester tester) async {
    const Color focusColor = Color(0xff001122);

    Color? getOverlayColor(Set<MaterialState> states) {
      return states.contains(MaterialState.focused) ? focusColor : null;
    }

    final FocusNode focusNode = FocusNode(debugLabel: 'FilledButton Node');
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: FilledButton(
          autofocus: true,
          style: ButtonStyle(
            overlayColor: MaterialStateProperty.resolveWith<Color?>(getOverlayColor),
          ),
          focusNode: focusNode,
          onPressed: () {},
          child: const Text('button'),
        ),
      ),
    );

    FocusManager.instance.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    await tester.pumpAndSettle();

    final RenderObject inkFeatures = tester.allRenderObjects.firstWhere(
      (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
    );
    expect(inkFeatures, paints..rect(color: focusColor));
    focusNode.dispose();
  });

  testWidgets('Does FilledButton contribute semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      Theme(
        data: ThemeData(useMaterial3: false),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: FilledButton(
              style: const ButtonStyle(
                // Specifying minimumSize to mimic the original minimumSize for
                // RaisedButton so that the semantics tree's rect and transform
                // match the original version of this test.
                minimumSize: MaterialStatePropertyAll<Size>(Size(88, 36)),
              ),
              onPressed: () {},
              child: const Text('ABC'),
            ),
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
              label: 'ABC',
              rect: const Rect.fromLTRB(0.0, 0.0, 88.0, 48.0),
              transform: Matrix4.translationValues(356.0, 276.0, 0.0),
              flags: <SemanticsFlag>[
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isButton,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isFocusable,
              ],
            ),
          ],
        ),
        ignoreId: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('FilledButton size is configurable by ThemeData.materialTapTargetSize', (
    WidgetTester tester,
  ) async {
    const ButtonStyle style = ButtonStyle(
      // Specifying minimumSize to mimic the original minimumSize for
      // RaisedButton so that the corresponding button size matches
      // the original version of this test.
      minimumSize: MaterialStatePropertyAll<Size>(Size(88, 36)),
    );

    Widget buildFrame(MaterialTapTargetSize tapTargetSize, Key key) {
      return Theme(
        data: ThemeData(useMaterial3: false, materialTapTargetSize: tapTargetSize),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: FilledButton(
              key: key,
              style: style,
              child: const SizedBox(width: 50.0, height: 8.0),
              onPressed: () {},
            ),
          ),
        ),
      );
    }

    final Key key1 = UniqueKey();
    await tester.pumpWidget(buildFrame(MaterialTapTargetSize.padded, key1));
    expect(tester.getSize(find.byKey(key1)), const Size(88.0, 48.0));

    final Key key2 = UniqueKey();
    await tester.pumpWidget(buildFrame(MaterialTapTargetSize.shrinkWrap, key2));
    expect(tester.getSize(find.byKey(key2)), const Size(88.0, 36.0));
  });

  testWidgets('FilledButton has no clip by default', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: FilledButton(
          onPressed: () {
            /* to make sure the button is enabled */
          },
          child: const Text('button'),
        ),
      ),
    );

    expect(tester.renderObject(find.byType(FilledButton)), paintsExactlyCountTimes(#clipPath, 0));
  });

  testWidgets('FilledButton responds to density changes.', (WidgetTester tester) async {
    const Key key = Key('test');
    const Key childKey = Key('test child');

    Future<void> buildTest(VisualDensity visualDensity, {bool useText = false}) async {
      return tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Directionality(
            textDirection: TextDirection.rtl,
            child: Center(
              child: FilledButton(
                style: ButtonStyle(
                  visualDensity: visualDensity,
                  // Specifying minimumSize to mimic the original minimumSize for
                  // RaisedButton so that the corresponding button size matches
                  // the original version of this test.
                  minimumSize: const MaterialStatePropertyAll<Size>(Size(88, 36)),
                ),
                key: key,
                onPressed: () {},
                child:
                    useText
                        ? const Text('Text', key: childKey)
                        : Container(
                          key: childKey,
                          width: 100,
                          height: 100,
                          color: const Color(0xffff0000),
                        ),
              ),
            ),
          ),
        ),
      );
    }

    await buildTest(VisualDensity.standard);
    final RenderBox box = tester.renderObject(find.byKey(key));
    Rect childRect = tester.getRect(find.byKey(childKey));
    await tester.pumpAndSettle();
    expect(box.size, equals(const Size(132, 100)));
    expect(childRect, equals(const Rect.fromLTRB(350, 250, 450, 350)));

    await buildTest(const VisualDensity(horizontal: 3.0, vertical: 3.0));
    await tester.pumpAndSettle();
    childRect = tester.getRect(find.byKey(childKey));
    expect(box.size, equals(const Size(156, 124)));
    expect(childRect, equals(const Rect.fromLTRB(350, 250, 450, 350)));

    await buildTest(const VisualDensity(horizontal: -3.0, vertical: -3.0));
    await tester.pumpAndSettle();
    childRect = tester.getRect(find.byKey(childKey));
    expect(box.size, equals(const Size(132, 100)));
    expect(childRect, equals(const Rect.fromLTRB(350, 250, 450, 350)));

    await buildTest(VisualDensity.standard, useText: true);
    await tester.pumpAndSettle();
    childRect = tester.getRect(find.byKey(childKey));
    expect(box.size, equals(const Size(88, 48)));
    expect(childRect, equals(const Rect.fromLTRB(372.0, 293.0, 428.0, 307.0)));

    await buildTest(const VisualDensity(horizontal: 3.0, vertical: 3.0), useText: true);
    await tester.pumpAndSettle();
    childRect = tester.getRect(find.byKey(childKey));
    expect(box.size, equals(const Size(112, 60)));
    expect(childRect, equals(const Rect.fromLTRB(372.0, 293.0, 428.0, 307.0)));

    await buildTest(const VisualDensity(horizontal: -3.0, vertical: -3.0), useText: true);
    await tester.pumpAndSettle();
    childRect = tester.getRect(find.byKey(childKey));
    expect(box.size, equals(const Size(88, 36)));
    expect(childRect, equals(const Rect.fromLTRB(372.0, 293.0, 428.0, 307.0)));
  });

  testWidgets('FilledButton.icon responds to applied padding', (WidgetTester tester) async {
    const Key buttonKey = Key('test');
    const Key labelKey = Key('label');
    await tester.pumpWidget(
      // When textDirection is set to TextDirection.ltr, the label appears on the
      // right side of the icon. This is important in determining whether the
      // horizontal padding is applied correctly later on
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: FilledButton.icon(
            key: buttonKey,
            style: const ButtonStyle(
              padding: MaterialStatePropertyAll<EdgeInsets>(EdgeInsets.fromLTRB(16, 5, 10, 12)),
            ),
            onPressed: () {},
            icon: const Icon(Icons.add),
            label: const Text('Hello', key: labelKey),
          ),
        ),
      ),
    );

    final Rect paddingRect = tester.getRect(find.byType(Padding));
    final Rect labelRect = tester.getRect(find.byKey(labelKey));
    final Rect iconRect = tester.getRect(find.byType(Icon));

    Matcher closeOnWeb(num value) {
      return kIsWeb ? closeTo(value, 1e-2) : equals(value);
    }

    // The right padding should be applied on the right of the label, whereas the
    // left padding should be applied on the left side of the icon.
    expect(paddingRect.right, equals(labelRect.right + 10));
    expect(paddingRect.left, equals(iconRect.left - 16));
    // Use the taller widget to check the top and bottom padding.
    final Rect tallerWidget = iconRect.height > labelRect.height ? iconRect : labelRect;
    expect(paddingRect.top, closeOnWeb(tallerWidget.top - 6.5));
    expect(paddingRect.bottom, closeOnWeb(tallerWidget.bottom + 13.5));
  });

  group('Default FilledButton padding for textScaleFactor, textDirection', () {
    const ValueKey<String> buttonKey = ValueKey<String>('button');
    const ValueKey<String> labelKey = ValueKey<String>('label');
    const ValueKey<String> iconKey = ValueKey<String>('icon');

    const List<double> textScaleFactorOptions = <double>[0.5, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 4.0];
    const List<TextDirection> textDirectionOptions = <TextDirection>[
      TextDirection.ltr,
      TextDirection.rtl,
    ];
    const List<Widget?> iconOptions = <Widget?>[null, Icon(Icons.add, size: 18, key: iconKey)];

    // Expected values for each textScaleFactor.
    final Map<double, double> paddingWithoutIconStart = <double, double>{
      0.5: 16,
      1: 16,
      1.25: 14,
      1.5: 12,
      2: 8,
      2.5: 6,
      3: 4,
      4: 4,
    };
    final Map<double, double> paddingWithoutIconEnd = <double, double>{
      0.5: 16,
      1: 16,
      1.25: 14,
      1.5: 12,
      2: 8,
      2.5: 6,
      3: 4,
      4: 4,
    };
    final Map<double, double> paddingWithIconStart = <double, double>{
      0.5: 12,
      1: 12,
      1.25: 11,
      1.5: 10,
      2: 8,
      2.5: 8,
      3: 8,
      4: 8,
    };
    final Map<double, double> paddingWithIconEnd = <double, double>{
      0.5: 16,
      1: 16,
      1.25: 14,
      1.5: 12,
      2: 8,
      2.5: 6,
      3: 4,
      4: 4,
    };
    final Map<double, double> paddingWithIconGap = <double, double>{
      0.5: 8,
      1: 8,
      1.25: 7,
      1.5: 6,
      2: 4,
      2.5: 4,
      3: 4,
      4: 4,
    };

    Rect globalBounds(RenderBox renderBox) {
      final Offset topLeft = renderBox.localToGlobal(Offset.zero);
      return topLeft & renderBox.size;
    }

    /// Computes the padding between two [Rect]s, one inside the other.
    EdgeInsets paddingBetween({required Rect parent, required Rect child}) {
      assert(parent.intersect(child) == child);
      return EdgeInsets.fromLTRB(
        child.left - parent.left,
        child.top - parent.top,
        parent.right - child.right,
        parent.bottom - child.bottom,
      );
    }

    for (final double textScaleFactor in textScaleFactorOptions) {
      for (final TextDirection textDirection in textDirectionOptions) {
        for (final Widget? icon in iconOptions) {
          final String testName = <String>[
            'FilledButton, text scale $textScaleFactor',
            if (icon != null) 'with icon',
            if (textDirection == TextDirection.rtl) 'RTL',
          ].join(', ');
          testWidgets(testName, (WidgetTester tester) async {
            await tester.pumpWidget(
              MaterialApp(
                theme: ThemeData(
                  useMaterial3: false,
                  filledButtonTheme: FilledButtonThemeData(
                    style: FilledButton.styleFrom(minimumSize: const Size(64, 36)),
                  ),
                ),
                home: Builder(
                  builder: (BuildContext context) {
                    return MediaQuery.withClampedTextScaling(
                      minScaleFactor: textScaleFactor,
                      maxScaleFactor: textScaleFactor,
                      child: Directionality(
                        textDirection: textDirection,
                        child: Scaffold(
                          body: Center(
                            child:
                                icon == null
                                    ? FilledButton(
                                      key: buttonKey,
                                      onPressed: () {},
                                      child: const Text('button', key: labelKey),
                                    )
                                    : FilledButton.icon(
                                      key: buttonKey,
                                      onPressed: () {},
                                      icon: icon,
                                      label: const Text('button', key: labelKey),
                                    ),
                          ),
                        ),
                      ),
                    );
                  },
                ),
              ),
            );

            final Element paddingElement = tester.element(
              find.descendant(of: find.byKey(buttonKey), matching: find.byType(Padding)),
            );
            expect(Directionality.of(paddingElement), textDirection);
            final Padding paddingWidget = paddingElement.widget as Padding;

            // Compute expected padding, and check.

            final double expectedStart =
                icon != null
                    ? paddingWithIconStart[textScaleFactor]!
                    : paddingWithoutIconStart[textScaleFactor]!;
            final double expectedEnd =
                icon != null
                    ? paddingWithIconEnd[textScaleFactor]!
                    : paddingWithoutIconEnd[textScaleFactor]!;
            final EdgeInsets expectedPadding = EdgeInsetsDirectional.fromSTEB(
              expectedStart,
              0,
              expectedEnd,
              0,
            ).resolve(textDirection);

            expect(paddingWidget.padding.resolve(textDirection), expectedPadding);

            // Measure padding in terms of the difference between the button and its label child
            // and check that.

            final RenderBox labelRenderBox = tester.renderObject<RenderBox>(find.byKey(labelKey));
            final Rect labelBounds = globalBounds(labelRenderBox);
            final RenderBox? iconRenderBox =
                icon == null ? null : tester.renderObject<RenderBox>(find.byKey(iconKey));
            final Rect? iconBounds = icon == null ? null : globalBounds(iconRenderBox!);
            final Rect childBounds =
                icon == null ? labelBounds : labelBounds.expandToInclude(iconBounds!);

            // We measure the `InkResponse` descendant of the button
            // element, because the button has a larger `RenderBox`
            // which accommodates the minimum tap target with a height
            // of 48.
            final RenderBox buttonRenderBox = tester.renderObject<RenderBox>(
              find.descendant(
                of: find.byKey(buttonKey),
                matching: find.byWidgetPredicate((Widget widget) => widget is InkResponse),
              ),
            );
            final Rect buttonBounds = globalBounds(buttonRenderBox);
            final EdgeInsets visuallyMeasuredPadding = paddingBetween(
              parent: buttonBounds,
              child: childBounds,
            );

            // Since there is a requirement of a minimum width of 64
            // and a minimum height of 36 on material buttons, the visual
            // padding of smaller buttons may not match their settings.
            // Therefore, we only test buttons that are large enough.
            if (buttonBounds.width > 64) {
              expect(visuallyMeasuredPadding.left, expectedPadding.left);
              expect(visuallyMeasuredPadding.right, expectedPadding.right);
            }

            if (buttonBounds.height > 36) {
              expect(visuallyMeasuredPadding.top, expectedPadding.top);
              expect(visuallyMeasuredPadding.bottom, expectedPadding.bottom);
            }

            // Check the gap between the icon and the label
            if (icon != null) {
              final double gapWidth =
                  textDirection == TextDirection.ltr
                      ? labelBounds.left - iconBounds!.right
                      : iconBounds!.left - labelBounds.right;
              expect(gapWidth, paddingWithIconGap[textScaleFactor]);
            }

            // Check the text's height - should be consistent with the textScaleFactor.
            final RenderBox textRenderObject = tester.renderObject<RenderBox>(
              find.descendant(
                of: find.byKey(labelKey),
                matching: find.byElementPredicate((Element element) => element.widget is RichText),
              ),
            );
            final double textHeight = textRenderObject.paintBounds.size.height;
            final double expectedTextHeight = 14 * textScaleFactor;
            expect(textHeight, moreOrLessEquals(expectedTextHeight, epsilon: 0.5));
          });
        }
      }
    }
  });

  testWidgets('Override FilledButton default padding', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.from(colorScheme: const ColorScheme.light()),
        home: Builder(
          builder: (BuildContext context) {
            return MediaQuery.withClampedTextScaling(
              minScaleFactor: 2,
              maxScaleFactor: 2,
              child: Scaffold(
                body: Center(
                  child: FilledButton(
                    style: FilledButton.styleFrom(padding: const EdgeInsets.all(22)),
                    onPressed: () {},
                    child: const Text('FilledButton'),
                  ),
                ),
              ),
            );
          },
        ),
      ),
    );

    final Padding paddingWidget = tester.widget<Padding>(
      find.descendant(of: find.byType(FilledButton), matching: find.byType(Padding)),
    );
    expect(paddingWidget.padding, const EdgeInsets.all(22));
  });

  testWidgets('Override theme fontSize changes padding', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.from(
          colorScheme: const ColorScheme.light(),
          textTheme: const TextTheme(labelLarge: TextStyle(fontSize: 28.0)),
        ),
        home: Builder(
          builder: (BuildContext context) {
            return Scaffold(
              body: Center(child: FilledButton(onPressed: () {}, child: const Text('text'))),
            );
          },
        ),
      ),
    );

    final Padding paddingWidget = tester.widget<Padding>(
      find.descendant(of: find.byType(FilledButton), matching: find.byType(Padding)),
    );
    expect(paddingWidget.padding, const EdgeInsets.symmetric(horizontal: 12));
  });

  testWidgets('M3 FilledButton has correct padding', (WidgetTester tester) async {
    final Key key = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.from(colorScheme: const ColorScheme.light()),
        home: Scaffold(
          body: Center(
            child: FilledButton(key: key, onPressed: () {}, child: const Text('FilledButton')),
          ),
        ),
      ),
    );

    final Padding paddingWidget = tester.widget<Padding>(
      find.descendant(of: find.byKey(key), matching: find.byType(Padding)),
    );
    expect(paddingWidget.padding, const EdgeInsets.symmetric(horizontal: 24));
  });

  testWidgets('M3 FilledButton.icon has correct padding', (WidgetTester tester) async {
    final Key key = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.from(colorScheme: const ColorScheme.light()),
        home: Scaffold(
          body: Center(
            child: FilledButton.icon(
              key: key,
              icon: const Icon(Icons.favorite),
              onPressed: () {},
              label: const Text('FilledButton'),
            ),
          ),
        ),
      ),
    );

    final Padding paddingWidget = tester.widget<Padding>(
      find.descendant(of: find.byKey(key), matching: find.byType(Padding)),
    );
    expect(paddingWidget.padding, const EdgeInsetsDirectional.fromSTEB(16.0, 0.0, 24.0, 0.0));
  });

  testWidgets('By default, FilledButton shape outline is defined by shape.side', (
    WidgetTester tester,
  ) async {
    const Color borderColor = Color(0xff4caf50);
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Center(
          child: FilledButton(
            style: FilledButton.styleFrom(
              shape: const RoundedRectangleBorder(
                borderRadius: BorderRadius.all(Radius.circular(16)),
                side: BorderSide(width: 10, color: borderColor),
              ),
              minimumSize: const Size(64, 36),
            ),
            onPressed: () {},
            child: const Text('button'),
          ),
        ),
      ),
    );

    expect(
      find.byType(FilledButton),
      paints..drrect(
        // Outer and inner rect that give the outline a width of 10.
        outer: RRect.fromLTRBR(0.0, 0.0, 116.0, 36.0, const Radius.circular(16)),
        inner: RRect.fromLTRBR(10.0, 10.0, 106.0, 26.0, const Radius.circular(16 - 10)),
        color: borderColor,
      ),
    );
  });

  testWidgets('Fixed size FilledButtons', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Column(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              FilledButton(
                style: FilledButton.styleFrom(fixedSize: const Size(100, 100)),
                onPressed: () {},
                child: const Text('100x100'),
              ),
              FilledButton(
                style: FilledButton.styleFrom(fixedSize: const Size.fromWidth(200)),
                onPressed: () {},
                child: const Text('200xh'),
              ),
              FilledButton(
                style: FilledButton.styleFrom(fixedSize: const Size.fromHeight(200)),
                onPressed: () {},
                child: const Text('wx200'),
              ),
            ],
          ),
        ),
      ),
    );

    expect(tester.getSize(find.widgetWithText(FilledButton, '100x100')), const Size(100, 100));
    expect(tester.getSize(find.widgetWithText(FilledButton, '200xh')).width, 200);
    expect(tester.getSize(find.widgetWithText(FilledButton, 'wx200')).height, 200);
  });

  testWidgets('FilledButton with NoSplash splashFactory paints nothing', (
    WidgetTester tester,
  ) async {
    Widget buildFrame({InteractiveInkFeatureFactory? splashFactory}) {
      return MaterialApp(
        home: Scaffold(
          body: Center(
            child: FilledButton(
              style: FilledButton.styleFrom(splashFactory: splashFactory),
              onPressed: () {},
              child: const Text('test'),
            ),
          ),
        ),
      );
    }

    // NoSplash.splashFactory, no splash circles drawn
    await tester.pumpWidget(buildFrame(splashFactory: NoSplash.splashFactory));
    {
      final TestGesture gesture = await tester.startGesture(tester.getCenter(find.text('test')));
      final MaterialInkController material = Material.of(tester.element(find.text('test')));
      await tester.pump(const Duration(milliseconds: 200));
      expect(material, paintsExactlyCountTimes(#drawCircle, 0));
      await gesture.up();
      await tester.pumpAndSettle();
    }

    // InkRipple.splashFactory, one splash circle drawn.
    await tester.pumpWidget(buildFrame(splashFactory: InkRipple.splashFactory));
    {
      final TestGesture gesture = await tester.startGesture(tester.getCenter(find.text('test')));
      final MaterialInkController material = Material.of(tester.element(find.text('test')));
      await tester.pump(const Duration(milliseconds: 200));
      expect(material, paintsExactlyCountTimes(#drawCircle, 1));
      await gesture.up();
      await tester.pumpAndSettle();
    }
  });

  testWidgets(
    'FilledButton uses InkSparkle only for Android non-web when useMaterial3 is true',
    (WidgetTester tester) async {
      final ThemeData theme = ThemeData();

      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: Center(child: FilledButton(onPressed: () {}, child: const Text('button'))),
        ),
      );

      final InkWell buttonInkWell = tester.widget<InkWell>(
        find.descendant(of: find.byType(FilledButton), matching: find.byType(InkWell)),
      );

      if (debugDefaultTargetPlatformOverride! == TargetPlatform.android && !kIsWeb) {
        expect(buttonInkWell.splashFactory, equals(InkSparkle.splashFactory));
      } else {
        expect(buttonInkWell.splashFactory, equals(InkRipple.splashFactory));
      }
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets('FilledButton.icon does not overflow', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/77815
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: SizedBox(
            width: 200,
            child: FilledButton.icon(
              onPressed: () {},
              icon: const Icon(Icons.add),
              label: const Text(
                // Much wider than 200
                'Lorem ipsum dolor sit amet, consectetur adipiscing elit. Ut a euismod nibh. Morbi laoreet purus.',
              ),
            ),
          ),
        ),
      ),
    );
    expect(tester.takeException(), null);
  });

  testWidgets('FilledButton.icon icon,label layout', (WidgetTester tester) async {
    final Key buttonKey = UniqueKey();
    final Key iconKey = UniqueKey();
    final Key labelKey = UniqueKey();
    final ButtonStyle style = FilledButton.styleFrom(
      padding: EdgeInsets.zero,
      visualDensity: VisualDensity.standard, // dx=0, dy=0
    );

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: SizedBox(
            width: 200,
            child: FilledButton.icon(
              key: buttonKey,
              style: style,
              onPressed: () {},
              icon: SizedBox(key: iconKey, width: 50, height: 100),
              label: SizedBox(key: labelKey, width: 50, height: 100),
            ),
          ),
        ),
      ),
    );

    // The button's label and icon are separated by a gap of 8:
    // 46 [icon 50] 8 [label 50] 46
    // The overall button width is 200. So:
    // icon.x = 46
    // label.x = 46 + 50 + 8 = 104

    expect(tester.getRect(find.byKey(buttonKey)), const Rect.fromLTRB(0.0, 0.0, 200.0, 100.0));
    expect(tester.getRect(find.byKey(iconKey)), const Rect.fromLTRB(46.0, 0.0, 96.0, 100.0));
    expect(tester.getRect(find.byKey(labelKey)), const Rect.fromLTRB(104.0, 0.0, 154.0, 100.0));
  });

  testWidgets('FilledButton maximumSize', (WidgetTester tester) async {
    final Key key0 = UniqueKey();
    final Key key1 = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: Center(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                FilledButton(
                  key: key0,
                  style: FilledButton.styleFrom(
                    minimumSize: const Size(24, 36),
                    maximumSize: const Size.fromWidth(64),
                  ),
                  onPressed: () {},
                  child: const Text('A B C D E F G H I J K L M N O P'),
                ),
                FilledButton.icon(
                  key: key1,
                  style: FilledButton.styleFrom(
                    minimumSize: const Size(24, 36),
                    maximumSize: const Size.fromWidth(104),
                  ),
                  onPressed: () {},
                  icon: Container(color: Colors.red, width: 32, height: 32),
                  label: const Text('A B C D E F G H I J K L M N O P'),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(tester.getSize(find.byKey(key0)), const Size(64.0, 224.0));
    expect(tester.getSize(find.byKey(key1)), const Size(104.0, 224.0));
  });

  testWidgets('Fixed size FilledButton, same as minimumSize == maximumSize', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Column(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              FilledButton(
                style: FilledButton.styleFrom(fixedSize: const Size(200, 200)),
                onPressed: () {},
                child: const Text('200x200'),
              ),
              FilledButton(
                style: FilledButton.styleFrom(
                  minimumSize: const Size(200, 200),
                  maximumSize: const Size(200, 200),
                ),
                onPressed: () {},
                child: const Text('200,200'),
              ),
            ],
          ),
        ),
      ),
    );

    expect(tester.getSize(find.widgetWithText(FilledButton, '200x200')), const Size(200, 200));
    expect(tester.getSize(find.widgetWithText(FilledButton, '200,200')), const Size(200, 200));
  });

  testWidgets('FilledButton changes mouse cursor when hovered', (WidgetTester tester) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: MouseRegion(
          cursor: SystemMouseCursors.forbidden,
          child: FilledButton(
            style: FilledButton.styleFrom(
              enabledMouseCursor: SystemMouseCursors.text,
              disabledMouseCursor: SystemMouseCursors.grab,
            ),
            onPressed: () {},
            child: const Text('button'),
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: Offset.zero);

    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );

    // Test cursor when disabled
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: MouseRegion(
          cursor: SystemMouseCursors.forbidden,
          child: FilledButton(
            style: FilledButton.styleFrom(
              enabledMouseCursor: SystemMouseCursors.text,
              disabledMouseCursor: SystemMouseCursors.grab,
            ),
            onPressed: null,
            child: const Text('button'),
          ),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.grab,
    );

    // Test default cursor
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: MouseRegion(
          cursor: SystemMouseCursors.forbidden,
          child: FilledButton(onPressed: () {}, child: const Text('button')),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );

    // Test default cursor when disabled
    await tester.pumpWidget(
      const Directionality(
        textDirection: TextDirection.ltr,
        child: MouseRegion(
          cursor: SystemMouseCursors.forbidden,
          child: FilledButton(onPressed: null, child: Text('button')),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );
  });

  testWidgets('FilledButton in SelectionArea changes mouse cursor when hovered', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/104595.
    await tester.pumpWidget(
      MaterialApp(
        home: SelectionArea(
          child: FilledButton(
            style: FilledButton.styleFrom(
              enabledMouseCursor: SystemMouseCursors.click,
              disabledMouseCursor: SystemMouseCursors.grab,
            ),
            onPressed: () {},
            child: const Text('button'),
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.byType(Text)));

    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );
  });

  testWidgets('Ink Response shape matches Material shape', (WidgetTester tester) async {
    Widget buildFrame({BorderSide? side}) {
      return MaterialApp(
        home: Scaffold(
          body: Center(
            child: FilledButton(
              style: FilledButton.styleFrom(
                side: side,
                shape: const RoundedRectangleBorder(
                  side: BorderSide(color: Color(0xff0000ff), width: 0),
                ),
              ),
              onPressed: () {},
              child: const Text('FilledButton'),
            ),
          ),
        ),
      );
    }

    const BorderSide borderSide = BorderSide(width: 10, color: Color(0xff00ff00));
    await tester.pumpWidget(buildFrame(side: borderSide));
    expect(
      tester.widget<InkWell>(find.byType(InkWell)).customBorder,
      const RoundedRectangleBorder(side: borderSide),
    );

    await tester.pumpWidget(buildFrame());
    await tester.pumpAndSettle();
    expect(
      tester.widget<InkWell>(find.byType(InkWell)).customBorder,
      const RoundedRectangleBorder(side: BorderSide(color: Color(0xff0000ff), width: 0.0)),
    );
  });

  testWidgets('FilledButton.styleFrom can be used to set foreground and background colors', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: FilledButton(
            style: FilledButton.styleFrom(
              foregroundColor: Colors.white,
              backgroundColor: Colors.purple,
            ),
            onPressed: () {},
            child: const Text('button'),
          ),
        ),
      ),
    );

    final Material material = tester.widget<Material>(
      find.descendant(of: find.byType(FilledButton), matching: find.byType(Material)),
    );
    expect(material.color, Colors.purple);
    expect(material.textStyle!.color, Colors.white);
  });

  Future<void> testStatesController(Widget? icon, WidgetTester tester) async {
    int count = 0;
    void valueChanged() {
      count += 1;
    }

    final MaterialStatesController controller = MaterialStatesController();
    addTearDown(controller.dispose);
    controller.addListener(valueChanged);

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child:
              icon == null
                  ? FilledButton(
                    statesController: controller,
                    onPressed: () {},
                    child: const Text('button'),
                  )
                  : FilledButton.icon(
                    statesController: controller,
                    onPressed: () {},
                    icon: icon,
                    label: const Text('button'),
                  ),
        ),
      ),
    );

    expect(controller.value, <MaterialState>{});
    expect(count, 0);

    final Offset center = tester.getCenter(find.byType(Text));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();

    expect(controller.value, <MaterialState>{MaterialState.hovered});
    expect(count, 1);

    await gesture.moveTo(Offset.zero);
    await tester.pumpAndSettle();

    expect(controller.value, <MaterialState>{});
    expect(count, 2);

    await gesture.moveTo(center);
    await tester.pumpAndSettle();

    expect(controller.value, <MaterialState>{MaterialState.hovered});
    expect(count, 3);

    await gesture.down(center);
    await tester.pumpAndSettle();

    expect(controller.value, <MaterialState>{MaterialState.hovered, MaterialState.pressed});
    expect(count, 4);

    await gesture.up();
    await tester.pumpAndSettle();

    expect(controller.value, <MaterialState>{MaterialState.hovered});
    expect(count, 5);

    await gesture.moveTo(Offset.zero);
    await tester.pumpAndSettle();

    expect(controller.value, <MaterialState>{});
    expect(count, 6);

    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(controller.value, <MaterialState>{MaterialState.hovered, MaterialState.pressed});
    expect(count, 8); // adds hovered and pressed - two changes

    // If the button is rebuilt disabled, then the pressed state is
    // removed.
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child:
              icon == null
                  ? FilledButton(
                    statesController: controller,
                    onPressed: null,
                    child: const Text('button'),
                  )
                  : FilledButton.icon(
                    statesController: controller,
                    onPressed: null,
                    icon: icon,
                    label: const Text('button'),
                  ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    expect(controller.value, <MaterialState>{MaterialState.hovered, MaterialState.disabled});
    expect(count, 10); // removes pressed and adds disabled - two changes
    await gesture.moveTo(Offset.zero);
    await tester.pumpAndSettle();
    expect(controller.value, <MaterialState>{MaterialState.disabled});
    expect(count, 11);
    await gesture.removePointer();
  }

  testWidgets('FilledButton statesController', (WidgetTester tester) async {
    testStatesController(null, tester);
  });

  testWidgets('FilledButton.icon statesController', (WidgetTester tester) async {
    testStatesController(const Icon(Icons.add), tester);
  });

  testWidgets('Disabled FilledButton statesController', (WidgetTester tester) async {
    int count = 0;
    void valueChanged() {
      count += 1;
    }

    final MaterialStatesController controller = MaterialStatesController();
    addTearDown(controller.dispose);
    controller.addListener(valueChanged);
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: FilledButton(
            statesController: controller,
            onPressed: null,
            child: const Text('button'),
          ),
        ),
      ),
    );
    expect(controller.value, <MaterialState>{MaterialState.disabled});
    expect(count, 1);
  });

  testWidgets('FilledButton backgroundBuilder and foregroundBuilder', (WidgetTester tester) async {
    const Color backgroundColor = Color(0xFF000011);
    const Color foregroundColor = Color(0xFF000022);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: FilledButton(
          style: FilledButton.styleFrom(
            backgroundBuilder: (BuildContext context, Set<MaterialState> states, Widget? child) {
              return DecoratedBox(
                decoration: const BoxDecoration(color: backgroundColor),
                child: child,
              );
            },
            foregroundBuilder: (BuildContext context, Set<MaterialState> states, Widget? child) {
              return DecoratedBox(
                decoration: const BoxDecoration(color: foregroundColor),
                child: child,
              );
            },
          ),
          onPressed: () {},
          child: const Text('button'),
        ),
      ),
    );

    BoxDecoration boxDecorationOf(Finder finder) {
      return tester.widget<DecoratedBox>(finder).decoration as BoxDecoration;
    }

    final Finder decorations = find.descendant(
      of: find.byType(FilledButton),
      matching: find.byType(DecoratedBox),
    );

    expect(boxDecorationOf(decorations.at(0)).color, backgroundColor);
    expect(boxDecorationOf(decorations.at(1)).color, foregroundColor);

    Text textChildOf(Finder finder) {
      return tester.widget<Text>(find.descendant(of: finder, matching: find.byType(Text)));
    }

    expect(textChildOf(decorations.at(0)).data, 'button');
    expect(textChildOf(decorations.at(1)).data, 'button');
  });

  testWidgets(
    'FilledButton backgroundBuilder drops button child and foregroundBuilder return value',
    (WidgetTester tester) async {
      const Color backgroundColor = Color(0xFF000011);
      const Color foregroundColor = Color(0xFF000022);

      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: FilledButton(
            style: FilledButton.styleFrom(
              backgroundBuilder: (BuildContext context, Set<MaterialState> states, Widget? child) {
                return const DecoratedBox(decoration: BoxDecoration(color: backgroundColor));
              },
              foregroundBuilder: (BuildContext context, Set<MaterialState> states, Widget? child) {
                return const DecoratedBox(decoration: BoxDecoration(color: foregroundColor));
              },
            ),
            onPressed: () {},
            child: const Text('button'),
          ),
        ),
      );

      final Finder background = find.descendant(
        of: find.byType(FilledButton),
        matching: find.byType(DecoratedBox),
      );

      expect(background, findsOneWidget);
      expect(find.text('button'), findsNothing);
    },
  );

  testWidgets('FilledButton foregroundBuilder drops button child', (WidgetTester tester) async {
    const Color foregroundColor = Color(0xFF000022);

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: FilledButton(
          style: FilledButton.styleFrom(
            foregroundBuilder: (BuildContext context, Set<MaterialState> states, Widget? child) {
              return const DecoratedBox(decoration: BoxDecoration(color: foregroundColor));
            },
          ),
          onPressed: () {},
          child: const Text('button'),
        ),
      ),
    );

    final Finder foreground = find.descendant(
      of: find.byType(FilledButton),
      matching: find.byType(DecoratedBox),
    );

    expect(foreground, findsOneWidget);
    expect(find.text('button'), findsNothing);
  });

  testWidgets('FilledButton foreground and background builders are applied to the correct states', (
    WidgetTester tester,
  ) async {
    Set<MaterialState> foregroundStates = <MaterialState>{};
    Set<MaterialState> backgroundStates = <MaterialState>{};
    final FocusNode focusNode = FocusNode();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: FilledButton(
              style: ButtonStyle(
                backgroundBuilder: (
                  BuildContext context,
                  Set<MaterialState> states,
                  Widget? child,
                ) {
                  backgroundStates = states;
                  return child!;
                },
                foregroundBuilder: (
                  BuildContext context,
                  Set<MaterialState> states,
                  Widget? child,
                ) {
                  foregroundStates = states;
                  return child!;
                },
              ),
              onPressed: () {},
              focusNode: focusNode,
              child: const Text('button'),
            ),
          ),
        ),
      ),
    );

    // Default.
    expect(backgroundStates.isEmpty, isTrue);
    expect(foregroundStates.isEmpty, isTrue);

    const Set<MaterialState> focusedStates = <MaterialState>{MaterialState.focused};
    const Set<MaterialState> focusedHoveredStates = <MaterialState>{
      MaterialState.focused,
      MaterialState.hovered,
    };
    const Set<MaterialState> focusedHoveredPressedStates = <MaterialState>{
      MaterialState.focused,
      MaterialState.hovered,
      MaterialState.pressed,
    };

    bool sameStates(Set<MaterialState> expectedValue, Set<MaterialState> actualValue) {
      return expectedValue.difference(actualValue).isEmpty &&
          actualValue.difference(expectedValue).isEmpty;
    }

    // Focused.
    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(sameStates(focusedStates, backgroundStates), isTrue);
    expect(sameStates(focusedStates, foregroundStates), isTrue);

    // Hovered.
    final Offset center = tester.getCenter(find.byType(FilledButton));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(sameStates(focusedHoveredStates, backgroundStates), isTrue);
    expect(sameStates(focusedHoveredStates, foregroundStates), isTrue);

    // Highlighted (pressed).
    await gesture.down(center);
    await tester.pump(); // Start the splash and highlight animations.
    await tester.pump(
      const Duration(milliseconds: 800),
    ); // Wait for splash and highlight to be well under way.
    expect(sameStates(focusedHoveredPressedStates, backgroundStates), isTrue);
    expect(sameStates(focusedHoveredPressedStates, foregroundStates), isTrue);

    focusNode.dispose();
  });

  testWidgets('Default FilledButton icon alignment', (WidgetTester tester) async {
    Widget buildWidget({required TextDirection textDirection}) {
      return MaterialApp(
        home: Directionality(
          textDirection: textDirection,
          child: Center(
            child: FilledButton.icon(
              onPressed: () {},
              icon: const Icon(Icons.add),
              label: const Text('button'),
            ),
          ),
        ),
      );
    }

    // Test default iconAlignment when textDirection is ltr.
    await tester.pumpWidget(buildWidget(textDirection: TextDirection.ltr));

    final Offset buttonTopLeft = tester.getTopLeft(find.byType(Material).last);
    final Offset iconTopLeft = tester.getTopLeft(find.byIcon(Icons.add));

    // The icon is aligned to the left of the button.
    expect(buttonTopLeft.dx, iconTopLeft.dx - 16.0); // 16.0 - padding between icon and button edge.

    // Test default iconAlignment when textDirection is rtl.
    await tester.pumpWidget(buildWidget(textDirection: TextDirection.rtl));

    final Offset buttonTopRight = tester.getTopRight(find.byType(Material).last);
    final Offset iconTopRight = tester.getTopRight(find.byIcon(Icons.add));

    // The icon is aligned to the right of the button.
    expect(
      buttonTopRight.dx,
      iconTopRight.dx + 16.0,
    ); // 16.0 - padding between icon and button edge.
  });

  testWidgets('FilledButton icon alignment can be customized', (WidgetTester tester) async {
    Widget buildWidget({
      required TextDirection textDirection,
      required IconAlignment iconAlignment,
    }) {
      return MaterialApp(
        home: Directionality(
          textDirection: textDirection,
          child: Center(
            child: FilledButton.icon(
              onPressed: () {},
              icon: const Icon(Icons.add),
              label: const Text('button'),
              iconAlignment: iconAlignment,
            ),
          ),
        ),
      );
    }

    // Test iconAlignment when textDirection is ltr.
    await tester.pumpWidget(
      buildWidget(textDirection: TextDirection.ltr, iconAlignment: IconAlignment.start),
    );

    Offset buttonTopLeft = tester.getTopLeft(find.byType(Material).last);
    Offset iconTopLeft = tester.getTopLeft(find.byIcon(Icons.add));

    // The icon is aligned to the left of the button.
    expect(buttonTopLeft.dx, iconTopLeft.dx - 16.0); // 16.0 - padding between icon and button edge.

    // Test iconAlignment when textDirection is ltr.
    await tester.pumpWidget(
      buildWidget(textDirection: TextDirection.ltr, iconAlignment: IconAlignment.end),
    );

    Offset buttonTopRight = tester.getTopRight(find.byType(Material).last);
    Offset iconTopRight = tester.getTopRight(find.byIcon(Icons.add));

    // The icon is aligned to the right of the button.
    expect(
      buttonTopRight.dx,
      iconTopRight.dx + 24.0,
    ); // 24.0 - padding between icon and button edge.

    // Test iconAlignment when textDirection is rtl.
    await tester.pumpWidget(
      buildWidget(textDirection: TextDirection.rtl, iconAlignment: IconAlignment.start),
    );

    buttonTopRight = tester.getTopRight(find.byType(Material).last);
    iconTopRight = tester.getTopRight(find.byIcon(Icons.add));

    // The icon is aligned to the right of the button.
    expect(
      buttonTopRight.dx,
      iconTopRight.dx + 16.0,
    ); // 16.0 - padding between icon and button edge.

    // Test iconAlignment when textDirection is rtl.
    await tester.pumpWidget(
      buildWidget(textDirection: TextDirection.rtl, iconAlignment: IconAlignment.end),
    );

    buttonTopLeft = tester.getTopLeft(find.byType(Material).last);
    iconTopLeft = tester.getTopLeft(find.byIcon(Icons.add));

    // The icon is aligned to the left of the button.
    expect(buttonTopLeft.dx, iconTopLeft.dx - 24.0); // 24.0 - padding between icon and button edge.
  });

  testWidgets('FilledButton icon alignment respects ButtonStyle.iconAlignment', (
    WidgetTester tester,
  ) async {
    Widget buildButton({IconAlignment? iconAlignment}) {
      return MaterialApp(
        home: Center(
          child: FilledButton.icon(
            style: ButtonStyle(iconAlignment: iconAlignment),
            onPressed: () {},
            icon: const Icon(Icons.add),
            label: const Text('button'),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildButton());

    final Offset buttonTopLeft = tester.getTopLeft(find.byType(Material).last);
    final Offset iconTopLeft = tester.getTopLeft(find.byIcon(Icons.add));

    expect(buttonTopLeft.dx, iconTopLeft.dx - 16.0);

    await tester.pumpWidget(buildButton(iconAlignment: IconAlignment.end));

    final Offset buttonTopRight = tester.getTopRight(find.byType(Material).last);
    final Offset iconTopRight = tester.getTopRight(find.byIcon(Icons.add));

    expect(buttonTopRight.dx, iconTopRight.dx + 24.0);
  });

  testWidgets('FilledButton tonal button icon alignment respects ButtonStyle.iconAlignment', (
    WidgetTester tester,
  ) async {
    Widget buildButton({IconAlignment? iconAlignment}) {
      return MaterialApp(
        home: Center(
          child: FilledButton.tonalIcon(
            style: ButtonStyle(iconAlignment: iconAlignment),
            onPressed: () {},
            icon: const Icon(Icons.add),
            label: const Text('button'),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildButton());

    final Offset buttonTopLeft = tester.getTopLeft(find.byType(Material).last);
    final Offset iconTopLeft = tester.getTopLeft(find.byIcon(Icons.add));

    expect(buttonTopLeft.dx, iconTopLeft.dx - 16.0);

    await tester.pumpWidget(buildButton(iconAlignment: IconAlignment.end));

    final Offset buttonTopRight = tester.getTopRight(find.byType(Material).last);
    final Offset iconTopRight = tester.getTopRight(find.byIcon(Icons.add));

    expect(buttonTopRight.dx, iconTopRight.dx + 24.0);
  });

  testWidgets('Tonal icon default iconAlignment', (WidgetTester tester) async {
    Widget buildWidget({required TextDirection textDirection}) {
      return MaterialApp(
        home: Directionality(
          textDirection: textDirection,
          child: Center(
            child: FilledButton.tonalIcon(
              onPressed: () {},
              icon: const Icon(Icons.add),
              label: const Text('button'),
            ),
          ),
        ),
      );
    }

    // Test default iconAlignment when textDirection is ltr.
    await tester.pumpWidget(buildWidget(textDirection: TextDirection.ltr));

    final Offset buttonTopLeft = tester.getTopLeft(find.byType(Material).last);
    final Offset iconTopLeft = tester.getTopLeft(find.byIcon(Icons.add));

    // The icon is aligned to the left of the button.
    expect(buttonTopLeft.dx, iconTopLeft.dx - 16.0); // 16.0 - padding between icon and button edge.

    // Test default iconAlignment when textDirection is rtl.
    await tester.pumpWidget(buildWidget(textDirection: TextDirection.rtl));

    final Offset buttonTopRight = tester.getTopRight(find.byType(Material).last);
    final Offset iconTopRight = tester.getTopRight(find.byIcon(Icons.add));

    // The icon is aligned to the right of the button.
    expect(
      buttonTopRight.dx,
      iconTopRight.dx + 16.0,
    ); // 16.0 - padding between icon and button edge.
  });

  testWidgets('Tonal icon iconAlignment can be customized', (WidgetTester tester) async {
    Widget buildWidget({
      required TextDirection textDirection,
      required IconAlignment iconAlignment,
    }) {
      return MaterialApp(
        home: Directionality(
          textDirection: textDirection,
          child: Center(
            child: FilledButton.tonalIcon(
              onPressed: () {},
              icon: const Icon(Icons.add),
              label: const Text('button'),
              iconAlignment: iconAlignment,
            ),
          ),
        ),
      );
    }

    // Test iconAlignment when textDirection is ltr.
    await tester.pumpWidget(
      buildWidget(textDirection: TextDirection.ltr, iconAlignment: IconAlignment.start),
    );

    Offset buttonTopLeft = tester.getTopLeft(find.byType(Material).last);
    Offset iconTopLeft = tester.getTopLeft(find.byIcon(Icons.add));

    // The icon is aligned to the left of the button.
    expect(buttonTopLeft.dx, iconTopLeft.dx - 16.0); // 16.0 - padding between icon and button edge.

    // Test iconAlignment when textDirection is ltr.
    await tester.pumpWidget(
      buildWidget(textDirection: TextDirection.ltr, iconAlignment: IconAlignment.end),
    );

    Offset buttonTopRight = tester.getTopRight(find.byType(Material).last);
    Offset iconTopRight = tester.getTopRight(find.byIcon(Icons.add));

    // The icon is aligned to the right of the button.
    expect(
      buttonTopRight.dx,
      iconTopRight.dx + 24.0,
    ); // 24.0 - padding between icon and button edge.

    // Test iconAlignment when textDirection is rtl.
    await tester.pumpWidget(
      buildWidget(textDirection: TextDirection.rtl, iconAlignment: IconAlignment.start),
    );

    buttonTopRight = tester.getTopRight(find.byType(Material).last);
    iconTopRight = tester.getTopRight(find.byIcon(Icons.add));

    // The icon is aligned to the right of the button.
    expect(
      buttonTopRight.dx,
      iconTopRight.dx + 16.0,
    ); // 16.0 - padding between icon and button edge.

    // Test iconAlignment when textDirection is rtl.
    await tester.pumpWidget(
      buildWidget(textDirection: TextDirection.rtl, iconAlignment: IconAlignment.end),
    );

    buttonTopLeft = tester.getTopLeft(find.byType(Material).last);
    iconTopLeft = tester.getTopLeft(find.byIcon(Icons.add));

    // The icon is aligned to the left of the button.
    expect(buttonTopLeft.dx, iconTopLeft.dx - 24.0); // 24.0 - padding between icon and button edge.
  });

  // Regression test for https://github.com/flutter/flutter/issues/154798.
  testWidgets('FilledButton.styleFrom can customize the button icon', (WidgetTester tester) async {
    const Color iconColor = Color(0xFFF000FF);
    const double iconSize = 32.0;
    const Color disabledIconColor = Color(0xFFFFF000);
    Widget buildButton({bool enabled = true}) {
      return MaterialApp(
        home: Material(
          child: Center(
            child: FilledButton.icon(
              style: FilledButton.styleFrom(
                iconColor: iconColor,
                iconSize: iconSize,
                iconAlignment: IconAlignment.end,
                disabledIconColor: disabledIconColor,
              ),
              onPressed: enabled ? () {} : null,
              icon: const Icon(Icons.add),
              label: const Text('Button'),
            ),
          ),
        ),
      );
    }

    // Test enabled button.
    await tester.pumpWidget(buildButton());
    expect(tester.getSize(find.byIcon(Icons.add)), const Size(iconSize, iconSize));
    expect(iconStyle(tester, Icons.add).color, iconColor);

    // Test disabled button.
    await tester.pumpWidget(buildButton(enabled: false));
    expect(iconStyle(tester, Icons.add).color, disabledIconColor);

    final Offset buttonTopRight = tester.getTopRight(find.byType(Material).last);
    final Offset iconTopRight = tester.getTopRight(find.byIcon(Icons.add));
    expect(buttonTopRight.dx, iconTopRight.dx + 24.0);
  });

  // Regression test for https://github.com/flutter/flutter/issues/162839.
  testWidgets('FilledButton icon uses provided foregroundColor over default icon color', (
    WidgetTester tester,
  ) async {
    const Color foregroundColor = Color(0xFFFF1234);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: Column(
              children: <Widget>[
                FilledButton.icon(
                  style: FilledButton.styleFrom(foregroundColor: foregroundColor),
                  onPressed: () {},
                  icon: const Icon(Icons.add),
                  label: const Text('Button'),
                ),
                FilledButton.tonalIcon(
                  style: FilledButton.styleFrom(foregroundColor: foregroundColor),
                  onPressed: () {},
                  icon: const Icon(Icons.mail),
                  label: const Text('Button'),
                ),
              ],
            ),
          ),
        ),
      ),
    );
    expect(iconStyle(tester, Icons.add).color, foregroundColor);
    expect(iconStyle(tester, Icons.mail).color, foregroundColor);
  });
}
