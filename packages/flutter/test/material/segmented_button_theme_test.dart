// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('SegmentedButtonThemeData copyWith, ==, hashCode basics', () {
    expect(const SegmentedButtonThemeData(), const SegmentedButtonThemeData().copyWith());
    expect(
      const SegmentedButtonThemeData().hashCode,
      const SegmentedButtonThemeData().copyWith().hashCode,
    );

    const SegmentedButtonThemeData custom = SegmentedButtonThemeData(
      style: ButtonStyle(backgroundColor: MaterialStatePropertyAll<Color>(Colors.green)),
      selectedIcon: Icon(Icons.error),
    );
    final SegmentedButtonThemeData copy = const SegmentedButtonThemeData().copyWith(
      style: custom.style,
      selectedIcon: custom.selectedIcon,
    );
    expect(copy, custom);
  });

  test('SegmentedButtonThemeData lerp special cases', () {
    expect(SegmentedButtonThemeData.lerp(null, null, 0), const SegmentedButtonThemeData());
    const SegmentedButtonThemeData theme = SegmentedButtonThemeData();
    expect(identical(SegmentedButtonThemeData.lerp(theme, theme, 0.5), theme), true);
  });

  testWidgets('Default SegmentedButtonThemeData debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const SegmentedButtonThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('With no other configuration, defaults are used', (WidgetTester tester) async {
    final ThemeData theme = ThemeData(useMaterial3: true);
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Center(
            child: SegmentedButton<int>(
              segments: const <ButtonSegment<int>>[
                ButtonSegment<int>(value: 1, label: Text('1')),
                ButtonSegment<int>(value: 2, label: Text('2')),
                ButtonSegment<int>(value: 3, label: Text('3'), enabled: false),
              ],
              selected: const <int>{2},
              onSelectionChanged: (Set<int> selected) {},
            ),
          ),
        ),
      ),
    );

    // Test first segment, should be enabled
    {
      final Finder text = find.text('1');
      final Finder parent = find.ancestor(of: text, matching: find.byType(Material)).first;
      final Finder selectedIcon = find.descendant(of: parent, matching: find.byIcon(Icons.check));
      final Material material = tester.widget<Material>(parent);
      expect(material.color, Colors.transparent);
      expect(material.shape, const RoundedRectangleBorder());
      expect(material.textStyle!.color, theme.colorScheme.onSurface);
      expect(material.textStyle!.fontFamily, 'Roboto');
      expect(material.textStyle!.fontSize, 14);
      expect(material.textStyle!.fontWeight, FontWeight.w500);
      expect(selectedIcon, findsNothing);
    }

    // Test second segment, should be enabled and selected
    {
      final Finder text = find.text('2');
      final Finder parent = find.ancestor(of: text, matching: find.byType(Material)).first;
      final Finder selectedIcon = find.descendant(of: parent, matching: find.byIcon(Icons.check));
      final Material material = tester.widget<Material>(parent);
      expect(material.color, theme.colorScheme.secondaryContainer);
      expect(material.shape, const RoundedRectangleBorder());
      expect(material.textStyle!.color, theme.colorScheme.onSecondaryContainer);
      expect(material.textStyle!.fontFamily, 'Roboto');
      expect(material.textStyle!.fontSize, 14);
      expect(material.textStyle!.fontWeight, FontWeight.w500);
      expect(selectedIcon, findsOneWidget);
    }

    // Test last segment, should be disabled
    {
      final Finder text = find.text('3');
      final Finder parent = find.ancestor(of: text, matching: find.byType(Material)).first;
      final Finder selectedIcon = find.descendant(of: parent, matching: find.byIcon(Icons.check));
      final Material material = tester.widget<Material>(parent);
      expect(material.color, Colors.transparent);
      expect(material.shape, const RoundedRectangleBorder());
      expect(material.textStyle!.color, theme.colorScheme.onSurface.withOpacity(0.38));
      expect(material.textStyle!.fontFamily, 'Roboto');
      expect(material.textStyle!.fontSize, 14);
      expect(material.textStyle!.fontWeight, FontWeight.w500);
      expect(selectedIcon, findsNothing);
    }
  });

  testWidgets('ThemeData.segmentedButtonTheme overrides defaults', (WidgetTester tester) async {
    final ThemeData theme = ThemeData(
      useMaterial3: true,
      segmentedButtonTheme: SegmentedButtonThemeData(
        style: ButtonStyle(
          backgroundColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
            if (states.contains(MaterialState.disabled)) {
              return Colors.blue;
            }
            if (states.contains(MaterialState.selected)) {
              return Colors.purple;
            }
            return null;
          }),
          foregroundColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
            if (states.contains(MaterialState.disabled)) {
              return Colors.yellow;
            }
            if (states.contains(MaterialState.selected)) {
              return Colors.brown;
            } else {
              return Colors.cyan;
            }
          }),
        ),
        selectedIcon: const Icon(Icons.error),
      ),
    );
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Center(
            child: SegmentedButton<int>(
              segments: const <ButtonSegment<int>>[
                ButtonSegment<int>(value: 1, label: Text('1')),
                ButtonSegment<int>(value: 2, label: Text('2')),
                ButtonSegment<int>(value: 3, label: Text('3'), enabled: false),
              ],
              selected: const <int>{2},
              onSelectionChanged: (Set<int> selected) {},
            ),
          ),
        ),
      ),
    );

    // Test first segment, should be enabled
    {
      final Finder text = find.text('1');
      final Finder parent = find.ancestor(of: text, matching: find.byType(Material)).first;
      final Finder selectedIcon = find.descendant(of: parent, matching: find.byIcon(Icons.error));
      final Material material = tester.widget<Material>(parent);
      expect(material.color, Colors.transparent);
      expect(material.shape, const RoundedRectangleBorder());
      expect(material.textStyle!.color, Colors.cyan);
      expect(material.textStyle!.fontFamily, 'Roboto');
      expect(material.textStyle!.fontSize, 14);
      expect(material.textStyle!.fontWeight, FontWeight.w500);
      expect(selectedIcon, findsNothing);
    }

    // Test second segment, should be enabled and selected
    {
      final Finder text = find.text('2');
      final Finder parent = find.ancestor(of: text, matching: find.byType(Material)).first;
      final Finder selectedIcon = find.descendant(of: parent, matching: find.byIcon(Icons.error));
      final Material material = tester.widget<Material>(parent);
      expect(material.color, Colors.purple);
      expect(material.shape, const RoundedRectangleBorder());
      expect(material.textStyle!.color, Colors.brown);
      expect(material.textStyle!.fontFamily, 'Roboto');
      expect(material.textStyle!.fontSize, 14);
      expect(material.textStyle!.fontWeight, FontWeight.w500);
      expect(selectedIcon, findsOneWidget);
    }

    // Test last segment, should be disabled
    {
      final Finder text = find.text('3');
      final Finder parent = find.ancestor(of: text, matching: find.byType(Material)).first;
      final Finder selectedIcon = find.descendant(of: parent, matching: find.byIcon(Icons.error));
      final Material material = tester.widget<Material>(parent);
      expect(material.color, Colors.blue);
      expect(material.shape, const RoundedRectangleBorder());
      expect(material.textStyle!.color, Colors.yellow);
      expect(material.textStyle!.fontFamily, 'Roboto');
      expect(material.textStyle!.fontSize, 14);
      expect(material.textStyle!.fontWeight, FontWeight.w500);
      expect(selectedIcon, findsNothing);
    }
  });

  testWidgets('SegmentedButtonTheme overrides ThemeData and defaults', (WidgetTester tester) async {
    final SegmentedButtonThemeData global = SegmentedButtonThemeData(
      style: ButtonStyle(
        backgroundColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
          if (states.contains(MaterialState.disabled)) {
            return Colors.blue;
          }
          if (states.contains(MaterialState.selected)) {
            return Colors.purple;
          }
          return null;
        }),
        foregroundColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
          if (states.contains(MaterialState.disabled)) {
            return Colors.yellow;
          }
          if (states.contains(MaterialState.selected)) {
            return Colors.brown;
          } else {
            return Colors.cyan;
          }
        }),
      ),
      selectedIcon: const Icon(Icons.error),
    );
    final SegmentedButtonThemeData segmentedTheme = SegmentedButtonThemeData(
      style: ButtonStyle(
        backgroundColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
          if (states.contains(MaterialState.disabled)) {
            return Colors.lightBlue;
          }
          if (states.contains(MaterialState.selected)) {
            return Colors.lightGreen;
          }
          return null;
        }),
        foregroundColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
          if (states.contains(MaterialState.disabled)) {
            return Colors.lime;
          }
          if (states.contains(MaterialState.selected)) {
            return Colors.amber;
          } else {
            return Colors.deepPurple;
          }
        }),
      ),
      selectedIcon: const Icon(Icons.plus_one),
    );
    final ThemeData theme = ThemeData(useMaterial3: true, segmentedButtonTheme: global);
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: SegmentedButtonTheme(
          data: segmentedTheme,
          child: Scaffold(
            body: Center(
              child: SegmentedButton<int>(
                segments: const <ButtonSegment<int>>[
                  ButtonSegment<int>(value: 1, label: Text('1')),
                  ButtonSegment<int>(value: 2, label: Text('2')),
                  ButtonSegment<int>(value: 3, label: Text('3'), enabled: false),
                ],
                selected: const <int>{2},
                onSelectionChanged: (Set<int> selected) {},
              ),
            ),
          ),
        ),
      ),
    );

    // Test first segment, should be enabled
    {
      final Finder text = find.text('1');
      final Finder parent = find.ancestor(of: text, matching: find.byType(Material)).first;
      final Finder selectedIcon = find.descendant(
        of: parent,
        matching: find.byIcon(Icons.plus_one),
      );
      final Material material = tester.widget<Material>(parent);
      expect(material.animationDuration, const Duration(milliseconds: 200));
      expect(material.borderRadius, null);
      expect(material.color, Colors.transparent);
      expect(material.shape, const RoundedRectangleBorder());
      expect(material.textStyle!.color, Colors.deepPurple);
      expect(material.textStyle!.fontFamily, 'Roboto');
      expect(material.textStyle!.fontSize, 14);
      expect(material.textStyle!.fontWeight, FontWeight.w500);
      expect(selectedIcon, findsNothing);
    }

    // Test second segment, should be enabled and selected
    {
      final Finder text = find.text('2');
      final Finder parent = find.ancestor(of: text, matching: find.byType(Material)).first;
      final Finder selectedIcon = find.descendant(
        of: parent,
        matching: find.byIcon(Icons.plus_one),
      );
      final Material material = tester.widget<Material>(parent);
      expect(material.animationDuration, const Duration(milliseconds: 200));
      expect(material.borderRadius, null);
      expect(material.color, Colors.lightGreen);
      expect(material.shape, const RoundedRectangleBorder());
      expect(material.textStyle!.color, Colors.amber);
      expect(material.textStyle!.fontFamily, 'Roboto');
      expect(material.textStyle!.fontSize, 14);
      expect(material.textStyle!.fontWeight, FontWeight.w500);
      expect(selectedIcon, findsOneWidget);
    }

    // Test last segment, should be disabled
    {
      final Finder text = find.text('3');
      final Finder parent = find.ancestor(of: text, matching: find.byType(Material)).first;
      final Finder selectedIcon = find.descendant(
        of: parent,
        matching: find.byIcon(Icons.plus_one),
      );
      final Material material = tester.widget<Material>(parent);
      expect(material.animationDuration, const Duration(milliseconds: 200));
      expect(material.borderRadius, null);
      expect(material.color, Colors.lightBlue);
      expect(material.shape, const RoundedRectangleBorder());
      expect(material.textStyle!.color, Colors.lime);
      expect(material.textStyle!.fontFamily, 'Roboto');
      expect(material.textStyle!.fontSize, 14);
      expect(material.textStyle!.fontWeight, FontWeight.w500);
      expect(selectedIcon, findsNothing);
    }
  });

  testWidgets('Widget parameters overrides SegmentedTheme, ThemeData and defaults', (
    WidgetTester tester,
  ) async {
    final SegmentedButtonThemeData global = SegmentedButtonThemeData(
      style: ButtonStyle(
        backgroundColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
          if (states.contains(MaterialState.disabled)) {
            return Colors.blue;
          }
          if (states.contains(MaterialState.selected)) {
            return Colors.purple;
          }
          return null;
        }),
        foregroundColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
          if (states.contains(MaterialState.disabled)) {
            return Colors.yellow;
          }
          if (states.contains(MaterialState.selected)) {
            return Colors.brown;
          } else {
            return Colors.cyan;
          }
        }),
      ),
      selectedIcon: const Icon(Icons.error),
    );
    final SegmentedButtonThemeData segmentedTheme = SegmentedButtonThemeData(
      style: ButtonStyle(
        backgroundColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
          if (states.contains(MaterialState.disabled)) {
            return Colors.lightBlue;
          }
          if (states.contains(MaterialState.selected)) {
            return Colors.lightGreen;
          }
          return null;
        }),
        foregroundColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
          if (states.contains(MaterialState.disabled)) {
            return Colors.lime;
          }
          if (states.contains(MaterialState.selected)) {
            return Colors.amber;
          } else {
            return Colors.deepPurple;
          }
        }),
      ),
      selectedIcon: const Icon(Icons.plus_one),
    );
    final ThemeData theme = ThemeData(useMaterial3: true, segmentedButtonTheme: global);
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: SegmentedButtonTheme(
          data: segmentedTheme,
          child: Scaffold(
            body: Center(
              child: SegmentedButton<int>(
                segments: const <ButtonSegment<int>>[
                  ButtonSegment<int>(value: 1, label: Text('1')),
                  ButtonSegment<int>(value: 2, label: Text('2')),
                  ButtonSegment<int>(value: 3, label: Text('3'), enabled: false),
                ],
                selected: const <int>{2},
                onSelectionChanged: (Set<int> selected) {},
                style: ButtonStyle(
                  backgroundColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
                    if (states.contains(MaterialState.disabled)) {
                      return Colors.black12;
                    }
                    if (states.contains(MaterialState.selected)) {
                      return Colors.grey;
                    }
                    return null;
                  }),
                  foregroundColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
                    if (states.contains(MaterialState.disabled)) {
                      return Colors.amberAccent;
                    }
                    if (states.contains(MaterialState.selected)) {
                      return Colors.deepOrange;
                    } else {
                      return Colors.deepPurpleAccent;
                    }
                  }),
                ),
                selectedIcon: const Icon(Icons.alarm),
              ),
            ),
          ),
        ),
      ),
    );

    // Test first segment, should be enabled
    {
      final Finder text = find.text('1');
      final Finder parent = find.ancestor(of: text, matching: find.byType(Material)).first;
      final Finder selectedIcon = find.descendant(of: parent, matching: find.byIcon(Icons.alarm));
      final Material material = tester.widget<Material>(parent);
      expect(material.animationDuration, const Duration(milliseconds: 200));
      expect(material.borderRadius, null);
      expect(material.color, Colors.transparent);
      expect(material.shape, const RoundedRectangleBorder());
      expect(material.textStyle!.color, Colors.deepPurpleAccent);
      expect(material.textStyle!.fontFamily, 'Roboto');
      expect(material.textStyle!.fontSize, 14);
      expect(material.textStyle!.fontWeight, FontWeight.w500);
      expect(selectedIcon, findsNothing);
    }

    // Test second segment, should be enabled and selected
    {
      final Finder text = find.text('2');
      final Finder parent = find.ancestor(of: text, matching: find.byType(Material)).first;
      final Finder selectedIcon = find.descendant(of: parent, matching: find.byIcon(Icons.alarm));
      final Material material = tester.widget<Material>(parent);
      expect(material.animationDuration, const Duration(milliseconds: 200));
      expect(material.borderRadius, null);
      expect(material.color, Colors.grey);
      expect(material.shape, const RoundedRectangleBorder());
      expect(material.textStyle!.color, Colors.deepOrange);
      expect(material.textStyle!.fontFamily, 'Roboto');
      expect(material.textStyle!.fontSize, 14);
      expect(material.textStyle!.fontWeight, FontWeight.w500);
      expect(selectedIcon, findsOneWidget);
    }

    // Test last segment, should be disabled
    {
      final Finder text = find.text('3');
      final Finder parent = find.ancestor(of: text, matching: find.byType(Material)).first;
      final Finder selectedIcon = find.descendant(of: parent, matching: find.byIcon(Icons.alarm));
      final Material material = tester.widget<Material>(parent);
      expect(material.animationDuration, const Duration(milliseconds: 200));
      expect(material.borderRadius, null);
      expect(material.color, Colors.black12);
      expect(material.shape, const RoundedRectangleBorder());
      expect(material.textStyle!.color, Colors.amberAccent);
      expect(material.textStyle!.fontFamily, 'Roboto');
      expect(material.textStyle!.fontSize, 14);
      expect(material.textStyle!.fontWeight, FontWeight.w500);
      expect(selectedIcon, findsNothing);
    }
  });

  testWidgets(
    'SegmentedButtonTheme SegmentedButton.styleFrom overlayColor overrides default overlay color',
    (WidgetTester tester) async {
      const Color overlayColor = Color(0xffff0000);
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            segmentedButtonTheme: SegmentedButtonThemeData(
              style: SegmentedButton.styleFrom(overlayColor: overlayColor),
            ),
          ),
          home: Scaffold(
            body: Center(
              child: SegmentedButton<int>(
                segments: const <ButtonSegment<int>>[
                  ButtonSegment<int>(value: 0, label: Text('Option 1')),
                  ButtonSegment<int>(value: 1, label: Text('Option 2')),
                ],
                onSelectionChanged: (Set<int> selected) {},
                selected: const <int>{1},
              ),
            ),
          ),
        ),
      );
      SplashController splashController(String buttonText) {
        final BuildContext context = tester.element(find.text(buttonText));
        return Material.of(context);
      }

      // Hovered selected segment,
      Offset center = tester.getCenter(find.text('Option 1'));
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      await gesture.moveTo(center);
      await tester.pumpAndSettle();
      expect(splashController('Option 1'), paints..rect(color: overlayColor.withOpacity(0.08)));

      // Hovered unselected segment,
      center = tester.getCenter(find.text('Option 2'));
      await gesture.moveTo(center);
      await tester.pumpAndSettle();
      expect(splashController('Option 2'), paints..rect(color: overlayColor.withOpacity(0.08)));

      // Highlighted unselected segment (pressed).
      center = tester.getCenter(find.text('Option 1'));
      await gesture.down(center);
      await tester.pumpAndSettle();
      expect(
        splashController('Option 1'),
        paints
          ..rect(color: overlayColor.withOpacity(0.08))
          ..rect(color: overlayColor.withOpacity(0.1)),
      );
      // Remove pressed and hovered states,
      await gesture.up();
      await tester.pumpAndSettle();
      await gesture.moveTo(const Offset(0, 50));
      await tester.pumpAndSettle();

      // Highlighted selected segment (pressed)
      center = tester.getCenter(find.text('Option 2'));
      await gesture.down(center);
      await tester.pumpAndSettle();
      expect(
        splashController('Option 2'),
        paints
          ..rect(color: overlayColor.withOpacity(0.08))
          ..rect(color: overlayColor.withOpacity(0.1)),
      );
      // Remove pressed and hovered states,
      await gesture.up();
      await tester.pumpAndSettle();
      await gesture.moveTo(const Offset(0, 50));
      await tester.pumpAndSettle();

      // Focused unselected segment.
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pumpAndSettle();
      expect(splashController('Option 1'), paints..rect(color: overlayColor.withOpacity(0.1)));

      // Focused selected segment.
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pumpAndSettle();
      expect(splashController('Option 2'), paints..rect(color: overlayColor.withOpacity(0.1)));
    },
  );
}
