// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('DividerThemeData copyWith, ==, hashCode basics', () {
    expect(const DividerThemeData(), const DividerThemeData().copyWith());
    expect(const DividerThemeData().hashCode, const DividerThemeData().copyWith().hashCode);
  });

  test('DividerThemeData null fields by default', () {
    const DividerThemeData dividerTheme = DividerThemeData();
    expect(dividerTheme.color, null);
    expect(dividerTheme.space, null);
    expect(dividerTheme.thickness, null);
    expect(dividerTheme.indent, null);
    expect(dividerTheme.endIndent, null);
  });

  testWidgets('Default DividerThemeData debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const DividerThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('DividerThemeData implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const DividerThemeData(
      color: Color(0xFFFFFFFF),
      space: 5.0,
      thickness: 4.0,
      indent: 3.0,
      endIndent: 2.0,
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[
      'color: ${const Color(0xffffffff)}',
      'space: 5.0',
      'thickness: 4.0',
      'indent: 3.0',
      'endIndent: 2.0',
    ]);
  });

  group('Material3 - Horizontal Divider', () {
    testWidgets('Passing no DividerThemeData returns defaults', (WidgetTester tester) async {
      final ThemeData theme = ThemeData();
      await tester.pumpWidget(MaterialApp(theme: theme, home: const Scaffold(body: Divider())));

      final RenderBox box = tester.firstRenderObject(find.byType(Divider));
      expect(box.size.height, 16.0);

      final Container container = tester.widget(find.byType(Container));
      final BoxDecoration decoration = container.decoration! as BoxDecoration;
      expect(decoration.border!.bottom.width, 1.0);

      expect(decoration.border!.bottom.color, theme.colorScheme.outlineVariant);

      final Rect dividerRect = tester.getRect(find.byType(Divider));
      final Rect lineRect = tester.getRect(find.byType(DecoratedBox));
      expect(lineRect.left, dividerRect.left);
      expect(lineRect.right, dividerRect.right);
    });

    testWidgets('Uses values from DividerThemeData', (WidgetTester tester) async {
      final DividerThemeData dividerTheme = _dividerTheme();
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(dividerTheme: dividerTheme),
          home: const Scaffold(body: Divider()),
        ),
      );

      final RenderBox box = tester.firstRenderObject(find.byType(Divider));
      expect(box.size.height, dividerTheme.space);

      final Container container = tester.widget(find.byType(Container));
      final BoxDecoration decoration = container.decoration! as BoxDecoration;
      expect(decoration.border!.bottom.width, dividerTheme.thickness);
      expect(decoration.border!.bottom.color, dividerTheme.color);

      final Rect dividerRect = tester.getRect(find.byType(Divider));
      final Rect lineRect = tester.getRect(find.byType(DecoratedBox));
      expect(lineRect.left, dividerRect.left + dividerTheme.indent!);
      expect(lineRect.right, dividerRect.right - dividerTheme.endIndent!);
    });

    testWidgets('DividerTheme overrides defaults', (WidgetTester tester) async {
      final DividerThemeData dividerTheme = _dividerTheme();
      await tester.pumpWidget(
        MaterialApp(home: Scaffold(body: DividerTheme(data: dividerTheme, child: const Divider()))),
      );

      final Container container = tester.widget(find.byType(Container));
      final BoxDecoration decoration = container.decoration! as BoxDecoration;
      expect(decoration.border!.bottom.width, dividerTheme.thickness);
      expect(decoration.border!.bottom.color, dividerTheme.color);
    });

    testWidgets('Widget properties take priority over theme', (WidgetTester tester) async {
      const Color color = Colors.purple;
      const double height = 10.0;
      const double thickness = 5.0;
      const double indent = 8.0;
      const double endIndent = 9.0;

      final DividerThemeData dividerTheme = _dividerTheme();
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(dividerTheme: dividerTheme),
          home: const Scaffold(
            body: Divider(
              color: color,
              height: height,
              thickness: thickness,
              indent: indent,
              endIndent: endIndent,
            ),
          ),
        ),
      );

      final RenderBox box = tester.firstRenderObject(find.byType(Divider));
      expect(box.size.height, height);

      final Container container = tester.widget(find.byType(Container));
      final BoxDecoration decoration = container.decoration! as BoxDecoration;
      expect(decoration.border!.bottom.width, thickness);
      expect(decoration.border!.bottom.color, color);

      final Rect dividerRect = tester.getRect(find.byType(Divider));
      final Rect lineRect = tester.getRect(find.byType(DecoratedBox));
      expect(lineRect.left, dividerRect.left + indent);
      expect(lineRect.right, dividerRect.right - endIndent);
    });
  });

  group('Material3 - Vertical Divider', () {
    testWidgets('Passing no DividerThemeData returns defaults', (WidgetTester tester) async {
      final ThemeData theme = ThemeData();
      await tester.pumpWidget(
        MaterialApp(theme: theme, home: const Scaffold(body: VerticalDivider())),
      );

      final RenderBox box = tester.firstRenderObject(find.byType(VerticalDivider));
      expect(box.size.width, 16.0);

      final Container container = tester.widget(find.byType(Container));
      final BoxDecoration decoration = container.decoration! as BoxDecoration;
      final Border border = decoration.border! as Border;
      expect(border.left.width, 1.0);

      expect(border.left.color, theme.colorScheme.outlineVariant);

      final Rect dividerRect = tester.getRect(find.byType(VerticalDivider));
      final Rect lineRect = tester.getRect(find.byType(DecoratedBox));
      expect(lineRect.top, dividerRect.top);
      expect(lineRect.bottom, dividerRect.bottom);
    });

    testWidgets('Uses values from DividerThemeData', (WidgetTester tester) async {
      final DividerThemeData dividerTheme = _dividerTheme();
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(dividerTheme: dividerTheme),
          home: const Scaffold(body: VerticalDivider()),
        ),
      );

      final RenderBox box = tester.firstRenderObject(find.byType(VerticalDivider));
      expect(box.size.width, dividerTheme.space);

      final Container container = tester.widget(find.byType(Container));
      final BoxDecoration decoration = container.decoration! as BoxDecoration;
      final Border border = decoration.border! as Border;
      expect(border.left.width, dividerTheme.thickness);
      expect(border.left.color, dividerTheme.color);

      final Rect dividerRect = tester.getRect(find.byType(VerticalDivider));
      final Rect lineRect = tester.getRect(find.byType(DecoratedBox));
      expect(lineRect.top, dividerRect.top + dividerTheme.indent!);
      expect(lineRect.bottom, dividerRect.bottom - dividerTheme.endIndent!);
    });

    testWidgets('DividerTheme overrides defaults', (WidgetTester tester) async {
      final DividerThemeData dividerTheme = _dividerTheme();
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(body: DividerTheme(data: dividerTheme, child: const VerticalDivider())),
        ),
      );

      final Container container = tester.widget(find.byType(Container));
      final BoxDecoration decoration = container.decoration! as BoxDecoration;
      final Border border = decoration.border! as Border;
      expect(border.left.width, dividerTheme.thickness);
      expect(border.left.color, dividerTheme.color);
    });

    testWidgets('Widget properties take priority over theme', (WidgetTester tester) async {
      const Color color = Colors.purple;
      const double width = 10.0;
      const double thickness = 5.0;
      const double indent = 8.0;
      const double endIndent = 9.0;

      final DividerThemeData dividerTheme = _dividerTheme();
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(dividerTheme: dividerTheme),
          home: const Scaffold(
            body: VerticalDivider(
              color: color,
              width: width,
              thickness: thickness,
              indent: indent,
              endIndent: endIndent,
            ),
          ),
        ),
      );

      final RenderBox box = tester.firstRenderObject(find.byType(VerticalDivider));
      expect(box.size.width, width);

      final Container container = tester.widget(find.byType(Container));
      final BoxDecoration decoration = container.decoration! as BoxDecoration;
      final Border border = decoration.border! as Border;
      expect(border.left.width, thickness);
      expect(border.left.color, color);

      final Rect dividerRect = tester.getRect(find.byType(VerticalDivider));
      final Rect lineRect = tester.getRect(find.byType(DecoratedBox));
      expect(lineRect.top, dividerRect.top + indent);
      expect(lineRect.bottom, dividerRect.bottom - endIndent);
    });
  });

  group('Material 2', () {
    // These tests are only relevant for Material 2. Once Material 2
    // support is deprecated and the APIs are removed, these tests
    // can be deleted.

    group('Material2 - Horizontal Divider', () {
      testWidgets('Passing no DividerThemeData returns defaults', (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(theme: ThemeData(useMaterial3: false), home: const Scaffold(body: Divider())),
        );

        final RenderBox box = tester.firstRenderObject(find.byType(Divider));
        expect(box.size.height, 16.0);

        final Container container = tester.widget(find.byType(Container));
        final BoxDecoration decoration = container.decoration! as BoxDecoration;
        expect(decoration.border!.bottom.width, 0.0);

        final ThemeData theme = ThemeData(useMaterial3: false);
        expect(decoration.border!.bottom.color, theme.dividerColor);

        final Rect dividerRect = tester.getRect(find.byType(Divider));
        final Rect lineRect = tester.getRect(find.byType(DecoratedBox));
        expect(lineRect.left, dividerRect.left);
        expect(lineRect.right, dividerRect.right);
      });

      testWidgets('DividerTheme overrides defaults', (WidgetTester tester) async {
        final DividerThemeData theme = _dividerTheme();
        await tester.pumpWidget(
          MaterialApp(home: Scaffold(body: DividerTheme(data: theme, child: const Divider()))),
        );

        final Container container = tester.widget(find.byType(Container));
        final BoxDecoration decoration = container.decoration! as BoxDecoration;
        expect(decoration.border!.bottom.width, theme.thickness);
        expect(decoration.border!.bottom.color, theme.color);
      });
    });

    group('Material2 - Vertical Divider', () {
      testWidgets('Passing no DividerThemeData returns defaults', (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(useMaterial3: false),
            home: const Scaffold(body: VerticalDivider()),
          ),
        );

        final RenderBox box = tester.firstRenderObject(find.byType(VerticalDivider));
        expect(box.size.width, 16.0);

        final Container container = tester.widget(find.byType(Container));
        final BoxDecoration decoration = container.decoration! as BoxDecoration;
        final Border border = decoration.border! as Border;
        expect(border.left.width, 0.0);

        final ThemeData theme = ThemeData(useMaterial3: false);
        expect(border.left.color, theme.dividerColor);

        final Rect dividerRect = tester.getRect(find.byType(VerticalDivider));
        final Rect lineRect = tester.getRect(find.byType(DecoratedBox));
        expect(lineRect.top, dividerRect.top);
        expect(lineRect.bottom, dividerRect.bottom);
      });
    });
  });
}

DividerThemeData _dividerTheme() {
  return const DividerThemeData(
    color: Colors.orange,
    space: 12.0,
    thickness: 2.0,
    indent: 7.0,
    endIndent: 5.0,
  );
}
