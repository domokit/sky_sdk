// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

class TestIcon extends StatefulWidget {
  const TestIcon({super.key});

  @override
  TestIconState createState() => TestIconState();
}

class TestIconState extends State<TestIcon> {
  late IconThemeData iconTheme;

  @override
  Widget build(BuildContext context) {
    iconTheme = IconTheme.of(context);
    return const Icon(Icons.add);
  }
}

class TestText extends StatefulWidget {
  const TestText(this.text, {super.key});

  final String text;

  @override
  TestTextState createState() => TestTextState();
}

class TestTextState extends State<TestText> {
  late TextStyle textStyle;

  @override
  Widget build(BuildContext context) {
    textStyle = DefaultTextStyle.of(context).style;
    return Text(widget.text);
  }
}

void main() {
  test('ListTileThemeData copyWith, ==, hashCode, basics', () {
    expect(const ListTileThemeData(), const ListTileThemeData().copyWith());
    expect(const ListTileThemeData().hashCode, const ListTileThemeData().copyWith().hashCode);
  });

  test('ListTileThemeData lerp special cases', () {
    expect(ListTileThemeData.lerp(null, null, 0), null);
    const ListTileThemeData data = ListTileThemeData();
    expect(identical(ListTileThemeData.lerp(data, data, 0.5), data), true);
  });

  test('ListTileThemeData defaults', () {
    const ListTileThemeData themeData = ListTileThemeData();
    expect(themeData.dense, null);
    expect(themeData.shape, null);
    expect(themeData.style, null);
    expect(themeData.selectedColor, null);
    expect(themeData.iconColor, null);
    expect(themeData.textColor, null);
    expect(themeData.titleTextStyle, null);
    expect(themeData.subtitleTextStyle, null);
    expect(themeData.leadingAndTrailingTextStyle, null);
    expect(themeData.contentPadding, null);
    expect(themeData.tileColor, null);
    expect(themeData.selectedTileColor, null);
    expect(themeData.horizontalTitleGap, null);
    expect(themeData.minVerticalPadding, null);
    expect(themeData.minLeadingWidth, null);
    expect(themeData.minTileHeight, null);
    expect(themeData.enableFeedback, null);
    expect(themeData.mouseCursor, null);
    expect(themeData.visualDensity, null);
    expect(themeData.titleAlignment, null);
  });

  testWidgets('Default ListTileThemeData debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const ListTileThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('ListTileThemeData implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const ListTileThemeData(
      dense: true,
      shape: StadiumBorder(),
      style: ListTileStyle.drawer,
      selectedColor: Color(0x00000001),
      iconColor: Color(0x00000002),
      textColor: Color(0x00000003),
      titleTextStyle: TextStyle(color: Color(0x00000004)),
      subtitleTextStyle: TextStyle(color: Color(0x00000005)),
      leadingAndTrailingTextStyle: TextStyle(color: Color(0x00000006)),
      contentPadding: EdgeInsets.all(100),
      tileColor: Color(0x00000007),
      selectedTileColor: Color(0x00000008),
      horizontalTitleGap: 200,
      minVerticalPadding: 300,
      minLeadingWidth: 400,
      minTileHeight: 30,
      enableFeedback: true,
      mouseCursor: MaterialStateMouseCursor.clickable,
      visualDensity: VisualDensity.comfortable,
      titleAlignment: ListTileTitleAlignment.top,
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(
      description,
      equalsIgnoringHashCodes(<String>[
        'dense: true',
        'shape: StadiumBorder(BorderSide(width: 0.0, style: none))',
        'style: drawer',
        'selectedColor: ${const Color(0x00000001)}',
        'iconColor: ${const Color(0x00000002)}',
        'textColor: ${const Color(0x00000003)}',
        'titleTextStyle: TextStyle(inherit: true, color: ${const Color(0x00000004)})',
        'subtitleTextStyle: TextStyle(inherit: true, color: ${const Color(0x00000005)})',
        'leadingAndTrailingTextStyle: TextStyle(inherit: true, color: ${const Color(0x00000006)})',
        'contentPadding: EdgeInsets.all(100.0)',
        'tileColor: ${const Color(0x00000007)}',
        'selectedTileColor: ${const Color(0x00000008)}',
        'horizontalTitleGap: 200.0',
        'minVerticalPadding: 300.0',
        'minLeadingWidth: 400.0',
        'minTileHeight: 30.0',
        'enableFeedback: true',
        'mouseCursor: WidgetStateMouseCursor(clickable)',
        'visualDensity: VisualDensity#00000(h: -1.0, v: -1.0)(horizontal: -1.0, vertical: -1.0)',
        'titleAlignment: ListTileTitleAlignment.top',
      ]),
    );
  });

  testWidgets('ListTileTheme backwards compatibility constructor', (WidgetTester tester) async {
    late ListTileThemeData theme;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: ListTileTheme(
            dense: true,
            shape: const StadiumBorder(),
            style: ListTileStyle.drawer,
            selectedColor: const Color(0x00000001),
            iconColor: const Color(0x00000002),
            textColor: const Color(0x00000003),
            contentPadding: const EdgeInsets.all(100),
            tileColor: const Color(0x00000004),
            selectedTileColor: const Color(0x00000005),
            horizontalTitleGap: 200,
            minVerticalPadding: 300,
            minLeadingWidth: 400,
            enableFeedback: true,
            mouseCursor: MaterialStateMouseCursor.clickable,
            child: Center(
              child: Builder(
                builder: (BuildContext context) {
                  theme = ListTileTheme.of(context);
                  return const Placeholder();
                },
              ),
            ),
          ),
        ),
      ),
    );

    expect(theme.dense, true);
    expect(theme.shape, const StadiumBorder());
    expect(theme.style, ListTileStyle.drawer);
    expect(theme.selectedColor, const Color(0x00000001));
    expect(theme.iconColor, const Color(0x00000002));
    expect(theme.textColor, const Color(0x00000003));
    expect(theme.contentPadding, const EdgeInsets.all(100));
    expect(theme.tileColor, const Color(0x00000004));
    expect(theme.selectedTileColor, const Color(0x00000005));
    expect(theme.horizontalTitleGap, 200);
    expect(theme.minVerticalPadding, 300);
    expect(theme.minLeadingWidth, 400);
    expect(theme.enableFeedback, true);
    expect(theme.mouseCursor, MaterialStateMouseCursor.clickable);
  });

  testWidgets('ListTileTheme', (WidgetTester tester) async {
    final Key listTileKey = UniqueKey();
    final Key titleKey = UniqueKey();
    final Key subtitleKey = UniqueKey();
    final Key leadingKey = UniqueKey();
    final Key trailingKey = UniqueKey();
    late ThemeData theme;

    Widget buildFrame({
      bool enabled = true,
      bool dense = false,
      bool selected = false,
      ShapeBorder? shape,
      Color? selectedColor,
      Color? iconColor,
      Color? textColor,
    }) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Center(
            child: ListTileTheme(
              data: ListTileThemeData(
                dense: dense,
                shape: shape,
                selectedColor: selectedColor,
                iconColor: iconColor,
                textColor: textColor,
                minVerticalPadding: 25.0,
                mouseCursor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
                  if (states.contains(MaterialState.disabled)) {
                    return SystemMouseCursors.forbidden;
                  }

                  return SystemMouseCursors.click;
                }),
                visualDensity: VisualDensity.compact,
                titleAlignment: ListTileTitleAlignment.bottom,
              ),
              child: Builder(
                builder: (BuildContext context) {
                  theme = Theme.of(context);
                  return ListTile(
                    key: listTileKey,
                    enabled: enabled,
                    selected: selected,
                    leading: TestIcon(key: leadingKey),
                    trailing: TestIcon(key: trailingKey),
                    title: TestText('title', key: titleKey),
                    subtitle: TestText('subtitle', key: subtitleKey),
                  );
                },
              ),
            ),
          ),
        ),
      );
    }

    const Color green = Color(0xFF00FF00);
    const Color red = Color(0xFFFF0000);
    const ShapeBorder roundedShape = RoundedRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(4.0)),
    );

    Color iconColor(Key key) => tester.state<TestIconState>(find.byKey(key)).iconTheme.color!;
    Color textColor(Key key) => tester.state<TestTextState>(find.byKey(key)).textStyle.color!;
    ShapeBorder inkWellBorder() =>
        tester
            .widget<InkWell>(
              find.descendant(of: find.byType(ListTile), matching: find.byType(InkWell)),
            )
            .customBorder!;

    // A selected ListTile's leading, trailing, and text get the primary color by default
    await tester.pumpWidget(buildFrame(selected: true));
    await tester.pump(const Duration(milliseconds: 300)); // DefaultTextStyle changes animate
    expect(iconColor(leadingKey), theme.primaryColor);
    expect(iconColor(trailingKey), theme.primaryColor);
    expect(textColor(titleKey), theme.primaryColor);
    expect(textColor(subtitleKey), theme.primaryColor);

    // A selected ListTile's leading, trailing, and text get the ListTileTheme's selectedColor
    await tester.pumpWidget(buildFrame(selected: true, selectedColor: green));
    await tester.pump(const Duration(milliseconds: 300)); // DefaultTextStyle changes animate
    expect(iconColor(leadingKey), green);
    expect(iconColor(trailingKey), green);
    expect(textColor(titleKey), green);
    expect(textColor(subtitleKey), green);

    // An unselected ListTile's leading and trailing get the ListTileTheme's iconColor
    // An unselected ListTile's title texts get the ListTileTheme's textColor
    await tester.pumpWidget(buildFrame(iconColor: red, textColor: green));
    await tester.pump(const Duration(milliseconds: 300)); // DefaultTextStyle changes animate
    expect(iconColor(leadingKey), red);
    expect(iconColor(trailingKey), red);
    expect(textColor(titleKey), green);
    expect(textColor(subtitleKey), green);

    // If the item is disabled it's rendered with the theme's disabled color.
    await tester.pumpWidget(buildFrame(enabled: false));
    await tester.pump(const Duration(milliseconds: 300)); // DefaultTextStyle changes animate
    expect(iconColor(leadingKey), theme.disabledColor);
    expect(iconColor(trailingKey), theme.disabledColor);
    expect(textColor(titleKey), theme.disabledColor);
    expect(textColor(subtitleKey), theme.disabledColor);

    // If the item is disabled it's rendered with the theme's disabled color.
    // Even if it's selected.
    await tester.pumpWidget(buildFrame(enabled: false, selected: true));
    await tester.pump(const Duration(milliseconds: 300)); // DefaultTextStyle changes animate
    expect(iconColor(leadingKey), theme.disabledColor);
    expect(iconColor(trailingKey), theme.disabledColor);
    expect(textColor(titleKey), theme.disabledColor);
    expect(textColor(subtitleKey), theme.disabledColor);

    // A selected ListTile's InkWell gets the ListTileTheme's shape
    await tester.pumpWidget(buildFrame(selected: true, shape: roundedShape));
    expect(inkWellBorder(), roundedShape);

    // Cursor updates when hovering disabled ListTile
    await tester.pumpWidget(buildFrame(enabled: false));
    final Offset listTile = tester.getCenter(find.byKey(titleKey));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(listTile);
    await tester.pumpAndSettle();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.forbidden,
    );

    // VisualDensity is respected
    final RenderBox box = tester.renderObject(find.byKey(listTileKey));
    expect(box.size, equals(const Size(800, 80.0)));

    // titleAlignment is respected.
    final Offset titleOffset = tester.getTopLeft(find.text('title'));
    final Offset leadingOffset = tester.getTopLeft(find.byKey(leadingKey));
    final Offset trailingOffset = tester.getTopRight(find.byKey(trailingKey));
    expect(leadingOffset.dy - titleOffset.dy, 6);
    expect(trailingOffset.dy - titleOffset.dy, 6);
  });

  testWidgets('ListTileTheme colors are applied to leading and trailing text widgets', (
    WidgetTester tester,
  ) async {
    final Key leadingKey = UniqueKey();
    final Key trailingKey = UniqueKey();

    const Color selectedColor = Colors.orange;
    const Color defaultColor = Colors.black;

    late ThemeData theme;
    Widget buildFrame({bool enabled = true, bool selected = false}) {
      return MaterialApp(
        home: Material(
          child: Center(
            child: ListTileTheme(
              data: const ListTileThemeData(selectedColor: selectedColor, textColor: defaultColor),
              child: Builder(
                builder: (BuildContext context) {
                  theme = Theme.of(context);
                  return ListTile(
                    enabled: enabled,
                    selected: selected,
                    leading: TestText('leading', key: leadingKey),
                    title: const TestText('title'),
                    trailing: TestText('trailing', key: trailingKey),
                  );
                },
              ),
            ),
          ),
        ),
      );
    }

    Color textColor(Key key) => tester.state<TestTextState>(find.byKey(key)).textStyle.color!;

    await tester.pumpWidget(buildFrame());
    // Enabled color should use ListTileTheme.textColor.
    expect(textColor(leadingKey), defaultColor);
    expect(textColor(trailingKey), defaultColor);

    await tester.pumpWidget(buildFrame(selected: true));
    // Wait for text color to animate.
    await tester.pumpAndSettle();
    // Selected color should use ListTileTheme.selectedColor.
    expect(textColor(leadingKey), selectedColor);
    expect(textColor(trailingKey), selectedColor);

    await tester.pumpWidget(buildFrame(enabled: false));
    // Wait for text color to animate.
    await tester.pumpAndSettle();
    // Disabled color should be ThemeData.disabledColor.
    expect(textColor(leadingKey), theme.disabledColor);
    expect(textColor(trailingKey), theme.disabledColor);
  });

  testWidgets(
    "Material3 - ListTile respects ListTileTheme's titleTextStyle, subtitleTextStyle & leadingAndTrailingTextStyle",
    (WidgetTester tester) async {
      const TextStyle titleTextStyle = TextStyle(
        fontSize: 23.0,
        color: Color(0xffff0000),
        fontStyle: FontStyle.italic,
      );
      const TextStyle subtitleTextStyle = TextStyle(
        fontSize: 20.0,
        color: Color(0xff00ff00),
        fontStyle: FontStyle.italic,
      );
      const TextStyle leadingAndTrailingTextStyle = TextStyle(
        fontSize: 18.0,
        color: Color(0xff0000ff),
        fontStyle: FontStyle.italic,
      );

      final ThemeData theme = ThemeData(
        useMaterial3: true,
        listTileTheme: const ListTileThemeData(
          titleTextStyle: titleTextStyle,
          subtitleTextStyle: subtitleTextStyle,
          leadingAndTrailingTextStyle: leadingAndTrailingTextStyle,
        ),
      );

      Widget buildFrame() {
        return MaterialApp(
          theme: theme,
          home: Material(
            child: Center(
              child: Builder(
                builder: (BuildContext context) {
                  return const ListTile(
                    leading: TestText('leading'),
                    title: TestText('title'),
                    subtitle: TestText('subtitle'),
                    trailing: TestText('trailing'),
                  );
                },
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildFrame());
      final RenderParagraph leading = _getTextRenderObject(tester, 'leading');
      expect(leading.text.style!.fontSize, leadingAndTrailingTextStyle.fontSize);
      expect(leading.text.style!.color, leadingAndTrailingTextStyle.color);
      expect(leading.text.style!.fontStyle, leadingAndTrailingTextStyle.fontStyle);
      final RenderParagraph title = _getTextRenderObject(tester, 'title');
      expect(title.text.style!.fontSize, titleTextStyle.fontSize);
      expect(title.text.style!.color, titleTextStyle.color);
      expect(title.text.style!.fontStyle, titleTextStyle.fontStyle);
      final RenderParagraph subtitle = _getTextRenderObject(tester, 'subtitle');
      expect(subtitle.text.style!.fontSize, subtitleTextStyle.fontSize);
      expect(subtitle.text.style!.color, subtitleTextStyle.color);
      expect(subtitle.text.style!.fontStyle, subtitleTextStyle.fontStyle);
      final RenderParagraph trailing = _getTextRenderObject(tester, 'trailing');
      expect(trailing.text.style!.fontSize, leadingAndTrailingTextStyle.fontSize);
      expect(trailing.text.style!.color, leadingAndTrailingTextStyle.color);
      expect(trailing.text.style!.fontStyle, leadingAndTrailingTextStyle.fontStyle);
    },
  );

  testWidgets(
    "Material2 - ListTile respects ListTileTheme's titleTextStyle, subtitleTextStyle & leadingAndTrailingTextStyle",
    (WidgetTester tester) async {
      const TextStyle titleTextStyle = TextStyle(
        fontSize: 23.0,
        color: Color(0xffff0000),
        fontStyle: FontStyle.italic,
      );
      const TextStyle subtitleTextStyle = TextStyle(
        fontSize: 20.0,
        color: Color(0xff00ff00),
        fontStyle: FontStyle.italic,
      );
      const TextStyle leadingAndTrailingTextStyle = TextStyle(
        fontSize: 18.0,
        color: Color(0xff0000ff),
        fontStyle: FontStyle.italic,
      );

      final ThemeData theme = ThemeData(
        useMaterial3: false,
        listTileTheme: const ListTileThemeData(
          titleTextStyle: titleTextStyle,
          subtitleTextStyle: subtitleTextStyle,
          leadingAndTrailingTextStyle: leadingAndTrailingTextStyle,
        ),
      );

      Widget buildFrame() {
        return MaterialApp(
          theme: theme,
          home: Material(
            child: Center(
              child: Builder(
                builder: (BuildContext context) {
                  return const ListTile(
                    leading: TestText('leading'),
                    title: TestText('title'),
                    subtitle: TestText('subtitle'),
                    trailing: TestText('trailing'),
                  );
                },
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildFrame());
      final RenderParagraph leading = _getTextRenderObject(tester, 'leading');
      expect(leading.text.style!.fontSize, leadingAndTrailingTextStyle.fontSize);
      expect(leading.text.style!.color, leadingAndTrailingTextStyle.color);
      expect(leading.text.style!.fontStyle, leadingAndTrailingTextStyle.fontStyle);
      final RenderParagraph title = _getTextRenderObject(tester, 'title');
      expect(title.text.style!.fontSize, titleTextStyle.fontSize);
      expect(title.text.style!.color, titleTextStyle.color);
      expect(title.text.style!.fontStyle, titleTextStyle.fontStyle);
      final RenderParagraph subtitle = _getTextRenderObject(tester, 'subtitle');
      expect(subtitle.text.style!.fontSize, subtitleTextStyle.fontSize);
      expect(subtitle.text.style!.color, subtitleTextStyle.color);
      expect(subtitle.text.style!.fontStyle, subtitleTextStyle.fontStyle);
      final RenderParagraph trailing = _getTextRenderObject(tester, 'trailing');
      expect(trailing.text.style!.fontSize, leadingAndTrailingTextStyle.fontSize);
      expect(trailing.text.style!.color, leadingAndTrailingTextStyle.color);
      expect(trailing.text.style!.fontStyle, leadingAndTrailingTextStyle.fontStyle);
    },
  );

  testWidgets(
    "Material3 - ListTile's titleTextStyle, subtitleTextStyle & leadingAndTrailingTextStyle are overridden by ListTile properties",
    (WidgetTester tester) async {
      final ThemeData theme = ThemeData(
        useMaterial3: true,
        listTileTheme: const ListTileThemeData(
          titleTextStyle: TextStyle(fontSize: 20.0),
          subtitleTextStyle: TextStyle(fontSize: 17.5),
          leadingAndTrailingTextStyle: TextStyle(fontSize: 15.0),
        ),
      );
      const TextStyle titleTextStyle = TextStyle(
        fontSize: 23.0,
        color: Color(0xffff0000),
        fontStyle: FontStyle.italic,
      );
      const TextStyle subtitleTextStyle = TextStyle(
        fontSize: 20.0,
        color: Color(0xff00ff00),
        fontStyle: FontStyle.italic,
      );
      const TextStyle leadingAndTrailingTextStyle = TextStyle(
        fontSize: 18.0,
        color: Color(0xff0000ff),
        fontStyle: FontStyle.italic,
      );

      Widget buildFrame() {
        return MaterialApp(
          theme: theme,
          home: Material(
            child: Center(
              child: Builder(
                builder: (BuildContext context) {
                  return const ListTile(
                    titleTextStyle: titleTextStyle,
                    subtitleTextStyle: subtitleTextStyle,
                    leadingAndTrailingTextStyle: leadingAndTrailingTextStyle,
                    leading: TestText('leading'),
                    title: TestText('title'),
                    subtitle: TestText('subtitle'),
                    trailing: TestText('trailing'),
                  );
                },
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildFrame());
      final RenderParagraph leading = _getTextRenderObject(tester, 'leading');
      expect(leading.text.style!.fontSize, leadingAndTrailingTextStyle.fontSize);
      expect(leading.text.style!.color, leadingAndTrailingTextStyle.color);
      expect(leading.text.style!.fontStyle, leadingAndTrailingTextStyle.fontStyle);
      final RenderParagraph title = _getTextRenderObject(tester, 'title');
      expect(title.text.style!.fontSize, titleTextStyle.fontSize);
      expect(title.text.style!.color, titleTextStyle.color);
      expect(title.text.style!.fontStyle, titleTextStyle.fontStyle);
      final RenderParagraph subtitle = _getTextRenderObject(tester, 'subtitle');
      expect(subtitle.text.style!.fontSize, subtitleTextStyle.fontSize);
      expect(subtitle.text.style!.color, subtitleTextStyle.color);
      expect(subtitle.text.style!.fontStyle, subtitleTextStyle.fontStyle);
      final RenderParagraph trailing = _getTextRenderObject(tester, 'trailing');
      expect(trailing.text.style!.fontSize, leadingAndTrailingTextStyle.fontSize);
      expect(trailing.text.style!.color, leadingAndTrailingTextStyle.color);
      expect(trailing.text.style!.fontStyle, leadingAndTrailingTextStyle.fontStyle);
    },
  );

  testWidgets(
    "Material2 - ListTile's titleTextStyle, subtitleTextStyle & leadingAndTrailingTextStyle are overridden by ListTile properties",
    (WidgetTester tester) async {
      final ThemeData theme = ThemeData(
        useMaterial3: false,
        listTileTheme: const ListTileThemeData(
          titleTextStyle: TextStyle(fontSize: 20.0),
          subtitleTextStyle: TextStyle(fontSize: 17.5),
          leadingAndTrailingTextStyle: TextStyle(fontSize: 15.0),
        ),
      );
      const TextStyle titleTextStyle = TextStyle(
        fontSize: 23.0,
        color: Color(0xffff0000),
        fontStyle: FontStyle.italic,
      );
      const TextStyle subtitleTextStyle = TextStyle(
        fontSize: 20.0,
        color: Color(0xff00ff00),
        fontStyle: FontStyle.italic,
      );
      const TextStyle leadingAndTrailingTextStyle = TextStyle(
        fontSize: 18.0,
        color: Color(0xff0000ff),
        fontStyle: FontStyle.italic,
      );

      Widget buildFrame() {
        return MaterialApp(
          theme: theme,
          home: Material(
            child: Center(
              child: Builder(
                builder: (BuildContext context) {
                  return const ListTile(
                    titleTextStyle: titleTextStyle,
                    subtitleTextStyle: subtitleTextStyle,
                    leadingAndTrailingTextStyle: leadingAndTrailingTextStyle,
                    leading: TestText('leading'),
                    title: TestText('title'),
                    subtitle: TestText('subtitle'),
                    trailing: TestText('trailing'),
                  );
                },
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildFrame());
      final RenderParagraph leading = _getTextRenderObject(tester, 'leading');
      expect(leading.text.style!.fontSize, leadingAndTrailingTextStyle.fontSize);
      expect(leading.text.style!.color, leadingAndTrailingTextStyle.color);
      expect(leading.text.style!.fontStyle, leadingAndTrailingTextStyle.fontStyle);
      final RenderParagraph title = _getTextRenderObject(tester, 'title');
      expect(title.text.style!.fontSize, titleTextStyle.fontSize);
      expect(title.text.style!.color, titleTextStyle.color);
      expect(title.text.style!.fontStyle, titleTextStyle.fontStyle);
      final RenderParagraph subtitle = _getTextRenderObject(tester, 'subtitle');
      expect(subtitle.text.style!.fontSize, subtitleTextStyle.fontSize);
      expect(subtitle.text.style!.color, subtitleTextStyle.color);
      expect(subtitle.text.style!.fontStyle, subtitleTextStyle.fontStyle);
      final RenderParagraph trailing = _getTextRenderObject(tester, 'trailing');
      expect(trailing.text.style!.fontSize, leadingAndTrailingTextStyle.fontSize);
      expect(trailing.text.style!.color, leadingAndTrailingTextStyle.color);
      expect(trailing.text.style!.fontStyle, leadingAndTrailingTextStyle.fontStyle);
    },
  );

  testWidgets("ListTile respects ListTileTheme's tileColor & selectedTileColor", (
    WidgetTester tester,
  ) async {
    late ListTileThemeData theme;
    bool isSelected = false;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: ListTileTheme(
            data: ListTileThemeData(
              tileColor: Colors.green.shade500,
              selectedTileColor: Colors.red.shade500,
            ),
            child: Center(
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  theme = ListTileTheme.of(context);
                  return ListTile(
                    selected: isSelected,
                    onTap: () {
                      setState(() => isSelected = !isSelected);
                    },
                    title: const Text('Title'),
                  );
                },
              ),
            ),
          ),
        ),
      ),
    );

    expect(find.byType(Material), paints..rect(color: theme.tileColor));

    // Tap on tile to change isSelected.
    await tester.tap(find.byType(ListTile));
    await tester.pumpAndSettle();

    expect(find.byType(Material), paints..rect(color: theme.selectedTileColor));
  });

  testWidgets(
    "ListTileTheme's tileColor & selectedTileColor are overridden by ListTile properties",
    (WidgetTester tester) async {
      bool isSelected = false;
      final Color tileColor = Colors.green.shade500;
      final Color selectedTileColor = Colors.red.shade500;

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: ListTileTheme(
              data: const ListTileThemeData(selectedTileColor: Colors.green, tileColor: Colors.red),
              child: Center(
                child: StatefulBuilder(
                  builder: (BuildContext context, StateSetter setState) {
                    return ListTile(
                      tileColor: tileColor,
                      selectedTileColor: selectedTileColor,
                      selected: isSelected,
                      onTap: () {
                        setState(() => isSelected = !isSelected);
                      },
                      title: const Text('Title'),
                    );
                  },
                ),
              ),
            ),
          ),
        ),
      );

      expect(find.byType(Material), paints..rect(color: tileColor));

      // Tap on tile to change isSelected.
      await tester.tap(find.byType(ListTile));
      await tester.pumpAndSettle();

      expect(find.byType(Material), paints..rect(color: selectedTileColor));
    },
  );

  testWidgets('ListTile uses ListTileTheme shape in a drawer', (WidgetTester tester) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/106303

    final GlobalKey<ScaffoldState> scaffoldKey = GlobalKey<ScaffoldState>();
    final ShapeBorder shapeBorder = RoundedRectangleBorder(
      borderRadius: BorderRadius.circular(20.0),
    );

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(listTileTheme: ListTileThemeData(shape: shapeBorder)),
        home: Scaffold(
          key: scaffoldKey,
          drawer: const Drawer(child: ListTile()),
          body: Container(),
        ),
      ),
    );
    await tester.pumpAndSettle();

    scaffoldKey.currentState!.openDrawer();
    // Start drawer animation.
    await tester.pump();

    final ShapeBorder? inkWellBorder =
        tester
            .widget<InkWell>(
              find.descendant(of: find.byType(ListTile), matching: find.byType(InkWell)),
            )
            .customBorder;
    // Test shape.
    expect(inkWellBorder, shapeBorder);
  });

  testWidgets('ListTile respects MaterialStateColor LisTileTheme.textColor', (
    WidgetTester tester,
  ) async {
    bool enabled = false;
    bool selected = false;
    const Color defaultColor = Colors.blue;
    const Color selectedColor = Colors.green;
    const Color disabledColor = Colors.red;

    final ThemeData theme = ThemeData(
      listTileTheme: ListTileThemeData(
        textColor: MaterialStateColor.resolveWith((Set<MaterialState> states) {
          if (states.contains(MaterialState.disabled)) {
            return disabledColor;
          }
          if (states.contains(MaterialState.selected)) {
            return selectedColor;
          }
          return defaultColor;
        }),
      ),
    );
    Widget buildFrame() {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: Builder(
              builder: (BuildContext context) {
                return ListTile(
                  enabled: enabled,
                  selected: selected,
                  title: const TestText('title'),
                  subtitle: const TestText('subtitle'),
                );
              },
            ),
          ),
        ),
      );
    }

    // Test disabled state.
    await tester.pumpWidget(buildFrame());
    RenderParagraph title = _getTextRenderObject(tester, 'title');
    expect(title.text.style!.color, disabledColor);

    // Test enabled state.
    enabled = true;
    await tester.pumpWidget(buildFrame());
    await tester.pumpAndSettle();
    title = _getTextRenderObject(tester, 'title');
    expect(title.text.style!.color, defaultColor);

    // Test selected state.
    selected = true;
    await tester.pumpWidget(buildFrame());
    await tester.pumpAndSettle();
    title = _getTextRenderObject(tester, 'title');
    expect(title.text.style!.color, selectedColor);
  });

  testWidgets('ListTile respects MaterialStateColor LisTileTheme.iconColor', (
    WidgetTester tester,
  ) async {
    bool enabled = false;
    bool selected = false;
    const Color defaultColor = Colors.blue;
    const Color selectedColor = Colors.green;
    const Color disabledColor = Colors.red;
    final Key leadingKey = UniqueKey();

    final ThemeData theme = ThemeData(
      listTileTheme: ListTileThemeData(
        iconColor: MaterialStateColor.resolveWith((Set<MaterialState> states) {
          if (states.contains(MaterialState.disabled)) {
            return disabledColor;
          }
          if (states.contains(MaterialState.selected)) {
            return selectedColor;
          }
          return defaultColor;
        }),
      ),
    );
    Widget buildFrame() {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: Builder(
              builder: (BuildContext context) {
                return ListTile(
                  enabled: enabled,
                  selected: selected,
                  leading: TestIcon(key: leadingKey),
                );
              },
            ),
          ),
        ),
      );
    }

    Color iconColor(Key key) => tester.state<TestIconState>(find.byKey(key)).iconTheme.color!;

    // Test disabled state.
    await tester.pumpWidget(buildFrame());
    expect(iconColor(leadingKey), disabledColor);

    // Test enabled state.
    enabled = true;
    await tester.pumpWidget(buildFrame());
    await tester.pumpAndSettle();
    expect(iconColor(leadingKey), defaultColor);

    // Test selected state.
    selected = true;
    await tester.pumpWidget(buildFrame());
    await tester.pumpAndSettle();
    expect(iconColor(leadingKey), selectedColor);
  });

  testWidgets('ListTileThemeData copyWith overrides all properties', (WidgetTester tester) async {
    // This is a regression test for https://github.com/flutter/flutter/issues/119734

    const ListTileThemeData original = ListTileThemeData(
      dense: true,
      shape: StadiumBorder(),
      style: ListTileStyle.drawer,
      selectedColor: Color(0x00000001),
      iconColor: Color(0x00000002),
      textColor: Color(0x00000003),
      titleTextStyle: TextStyle(color: Color(0x00000004)),
      subtitleTextStyle: TextStyle(color: Color(0x00000005)),
      leadingAndTrailingTextStyle: TextStyle(color: Color(0x00000006)),
      contentPadding: EdgeInsets.all(100),
      tileColor: Color(0x00000007),
      selectedTileColor: Color(0x00000008),
      horizontalTitleGap: 200,
      minVerticalPadding: 300,
      minLeadingWidth: 400,
      minTileHeight: 30,
      enableFeedback: true,
      titleAlignment: ListTileTitleAlignment.bottom,
    );

    final ListTileThemeData copy = original.copyWith(
      dense: false,
      shape: const RoundedRectangleBorder(),
      style: ListTileStyle.list,
      selectedColor: const Color(0x00000009),
      iconColor: const Color(0x0000000A),
      textColor: const Color(0x0000000B),
      titleTextStyle: const TextStyle(color: Color(0x0000000C)),
      subtitleTextStyle: const TextStyle(color: Color(0x0000000D)),
      leadingAndTrailingTextStyle: const TextStyle(color: Color(0x0000000E)),
      contentPadding: const EdgeInsets.all(500),
      tileColor: const Color(0x0000000F),
      selectedTileColor: const Color(0x00000010),
      horizontalTitleGap: 600,
      minVerticalPadding: 700,
      minLeadingWidth: 800,
      minTileHeight: 80,
      enableFeedback: false,
      titleAlignment: ListTileTitleAlignment.top,
    );

    expect(copy.dense, false);
    expect(copy.shape, const RoundedRectangleBorder());
    expect(copy.style, ListTileStyle.list);
    expect(copy.selectedColor, const Color(0x00000009));
    expect(copy.iconColor, const Color(0x0000000A));
    expect(copy.textColor, const Color(0x0000000B));
    expect(copy.titleTextStyle, const TextStyle(color: Color(0x0000000C)));
    expect(copy.subtitleTextStyle, const TextStyle(color: Color(0x0000000D)));
    expect(copy.leadingAndTrailingTextStyle, const TextStyle(color: Color(0x0000000E)));
    expect(copy.contentPadding, const EdgeInsets.all(500));
    expect(copy.tileColor, const Color(0x0000000F));
    expect(copy.selectedTileColor, const Color(0x00000010));
    expect(copy.horizontalTitleGap, 600);
    expect(copy.minVerticalPadding, 700);
    expect(copy.minLeadingWidth, 800);
    expect(copy.minTileHeight, 80);
    expect(copy.enableFeedback, false);
    expect(copy.titleAlignment, ListTileTitleAlignment.top);
  });

  testWidgets('ListTileTheme.titleAlignment is overridden by ListTile.titleAlignment', (
    WidgetTester tester,
  ) async {
    final Key leadingKey = GlobalKey();
    final Key trailingKey = GlobalKey();
    const String titleText = '\nHeadline Text\n';
    const String subtitleText = '\nSupporting Text\n';

    Widget buildFrame({ListTileTitleAlignment? alignment}) {
      return MaterialApp(
        theme: ThemeData(
          useMaterial3: true,
          listTileTheme: const ListTileThemeData(titleAlignment: ListTileTitleAlignment.center),
        ),
        home: Material(
          child: Center(
            child: ListTile(
              titleAlignment: ListTileTitleAlignment.top,
              leading: SizedBox(key: leadingKey, width: 24.0, height: 24.0),
              title: const Text(titleText),
              subtitle: const Text(subtitleText),
              trailing: SizedBox(key: trailingKey, width: 24.0, height: 24.0),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame());
    final Offset tileOffset = tester.getTopLeft(find.byType(ListTile));
    final Offset leadingOffset = tester.getTopLeft(find.byKey(leadingKey));
    final Offset trailingOffset = tester.getTopRight(find.byKey(trailingKey));
    expect(leadingOffset.dy - tileOffset.dy, 8.0);
    expect(trailingOffset.dy - tileOffset.dy, 8.0);
  });

  testWidgets('ListTileTheme.merge supports all properties', (WidgetTester tester) async {
    Widget buildFrame() {
      return MaterialApp(
        theme: ThemeData(
          listTileTheme: const ListTileThemeData(
            dense: true,
            shape: StadiumBorder(),
            style: ListTileStyle.drawer,
            selectedColor: Color(0x00000001),
            iconColor: Color(0x00000002),
            textColor: Color(0x00000003),
            titleTextStyle: TextStyle(color: Color(0x00000004)),
            subtitleTextStyle: TextStyle(color: Color(0x00000005)),
            leadingAndTrailingTextStyle: TextStyle(color: Color(0x00000006)),
            contentPadding: EdgeInsets.all(100),
            tileColor: Color(0x00000007),
            selectedTileColor: Color(0x00000008),
            horizontalTitleGap: 200,
            minVerticalPadding: 300,
            minLeadingWidth: 400,
            minTileHeight: 30,
            enableFeedback: true,
            titleAlignment: ListTileTitleAlignment.bottom,
            mouseCursor: MaterialStateMouseCursor.textable,
            visualDensity: VisualDensity.comfortable,
          ),
        ),
        home: Material(
          child: Center(
            child: Builder(
              builder: (BuildContext context) {
                return ListTileTheme.merge(
                  dense: false,
                  shape: const RoundedRectangleBorder(),
                  style: ListTileStyle.list,
                  selectedColor: const Color(0x00000009),
                  iconColor: const Color(0x0000000A),
                  textColor: const Color(0x0000000B),
                  titleTextStyle: const TextStyle(color: Color(0x0000000C)),
                  subtitleTextStyle: const TextStyle(color: Color(0x0000000D)),
                  leadingAndTrailingTextStyle: const TextStyle(color: Color(0x0000000E)),
                  contentPadding: const EdgeInsets.all(500),
                  tileColor: const Color(0x0000000F),
                  selectedTileColor: const Color(0x00000010),
                  horizontalTitleGap: 600,
                  minVerticalPadding: 700,
                  minLeadingWidth: 800,
                  minTileHeight: 80,
                  enableFeedback: false,
                  titleAlignment: ListTileTitleAlignment.top,
                  mouseCursor: MaterialStateMouseCursor.clickable,
                  visualDensity: VisualDensity.compact,
                  child: const ListTile(),
                );
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame());
    final ListTileThemeData theme = ListTileTheme.of(tester.element(find.byType(ListTile)));
    expect(theme.dense, false);
    expect(theme.shape, const RoundedRectangleBorder());
    expect(theme.style, ListTileStyle.list);
    expect(theme.selectedColor, const Color(0x00000009));
    expect(theme.iconColor, const Color(0x0000000A));
    expect(theme.textColor, const Color(0x0000000B));
    expect(theme.titleTextStyle, const TextStyle(color: Color(0x0000000C)));
    expect(theme.subtitleTextStyle, const TextStyle(color: Color(0x0000000D)));
    expect(theme.leadingAndTrailingTextStyle, const TextStyle(color: Color(0x0000000E)));
    expect(theme.contentPadding, const EdgeInsets.all(500));
    expect(theme.tileColor, const Color(0x0000000F));
    expect(theme.selectedTileColor, const Color(0x00000010));
    expect(theme.horizontalTitleGap, 600);
    expect(theme.minVerticalPadding, 700);
    expect(theme.minLeadingWidth, 800);
    expect(theme.minTileHeight, 80);
    expect(theme.enableFeedback, false);
    expect(theme.titleAlignment, ListTileTitleAlignment.top);
    expect(theme.mouseCursor, MaterialStateMouseCursor.clickable);
    expect(theme.visualDensity, VisualDensity.compact);
  });
}

RenderParagraph _getTextRenderObject(WidgetTester tester, String text) {
  return tester.renderObject(find.descendant(of: find.byType(ListTile), matching: find.text(text)));
}
