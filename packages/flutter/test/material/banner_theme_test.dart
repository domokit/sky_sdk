// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('MaterialBannerThemeData copyWith, ==, hashCode basics', () {
    expect(const MaterialBannerThemeData(), const MaterialBannerThemeData().copyWith());
    expect(
      const MaterialBannerThemeData().hashCode,
      const MaterialBannerThemeData().copyWith().hashCode,
    );
  });

  test('MaterialBannerThemeData null fields by default', () {
    const MaterialBannerThemeData bannerTheme = MaterialBannerThemeData();
    expect(bannerTheme.backgroundColor, null);
    expect(bannerTheme.surfaceTintColor, null);
    expect(bannerTheme.shadowColor, null);
    expect(bannerTheme.dividerColor, null);
    expect(bannerTheme.contentTextStyle, null);
    expect(bannerTheme.elevation, null);
    expect(bannerTheme.padding, null);
    expect(bannerTheme.leadingPadding, null);
  });

  testWidgets('Default MaterialBannerThemeData debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const MaterialBannerThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('MaterialBannerThemeData implements debugFillProperties', (
    WidgetTester tester,
  ) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const MaterialBannerThemeData(
      backgroundColor: Color(0xfffffff0),
      surfaceTintColor: Color(0xfffffff1),
      shadowColor: Color(0xfffffff2),
      dividerColor: Color(0xfffffff3),
      contentTextStyle: TextStyle(color: Color(0xfffffff4)),
      elevation: 4.0,
      padding: EdgeInsets.all(20.0),
      leadingPadding: EdgeInsets.only(left: 8.0),
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[
      'backgroundColor: ${const Color(0xfffffff0)}',
      'surfaceTintColor: ${const Color(0xfffffff1)}',
      'shadowColor: ${const Color(0xfffffff2)}',
      'dividerColor: ${const Color(0xfffffff3)}',
      'contentTextStyle: TextStyle(inherit: true, color: ${const Color(0xfffffff4)})',
      'elevation: 4.0',
      'padding: EdgeInsets.all(20.0)',
      'leadingPadding: EdgeInsets(8.0, 0.0, 0.0, 0.0)',
    ]);
  });

  testWidgets('Material3 - Passing no MaterialBannerThemeData returns defaults', (
    WidgetTester tester,
  ) async {
    const String contentText = 'Content';
    final ThemeData theme = ThemeData();
    late final ThemeData localizedTheme;

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        builder: (BuildContext context, Widget? child) {
          localizedTheme = Theme.of(context);
          return child!;
        },
        home: Scaffold(
          body: MaterialBanner(
            content: const Text(contentText),
            leading: const Icon(Icons.umbrella),
            actions: <Widget>[TextButton(child: const Text('Action'), onPressed: () {})],
          ),
        ),
      ),
    );

    final Material material = _getMaterialFromText(tester, contentText);
    expect(material.color, theme.colorScheme.surfaceContainerLow);
    expect(material.surfaceTintColor, Colors.transparent);
    expect(material.shadowColor, null);
    expect(material.elevation, 0.0);

    final RenderParagraph content = _getTextRenderObjectFromDialog(tester, contentText);
    expect(content.text.style, localizedTheme.textTheme.bodyMedium);

    final Offset rowTopLeft = tester.getTopLeft(find.byType(Row));
    final Offset materialTopLeft = tester.getTopLeft(_materialFinder());
    final Offset leadingTopLeft = tester.getTopLeft(find.byIcon(Icons.umbrella));
    expect(rowTopLeft.dy - materialTopLeft.dy, 2.0); // Default single line top padding.
    expect(rowTopLeft.dx - materialTopLeft.dx, 16.0); // Default single line start padding.
    expect(leadingTopLeft.dy - materialTopLeft.dy, 16); // Default leading padding.
    expect(leadingTopLeft.dx - materialTopLeft.dx, 16); // Default leading padding.

    final Divider divider = tester.widget<Divider>(find.byType(Divider));
    expect(divider.color, theme.colorScheme.outlineVariant);
  });

  testWidgets(
    'Material3 - Passing no MaterialBannerThemeData returns defaults when presented by ScaffoldMessenger',
    (WidgetTester tester) async {
      const String contentText = 'Content';
      const Key tapTarget = Key('tap-target');
      final ThemeData theme = ThemeData();
      late final ThemeData localizedTheme;

      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                localizedTheme = Theme.of(context);
                return GestureDetector(
                  key: tapTarget,
                  onTap: () {
                    ScaffoldMessenger.of(context).showMaterialBanner(
                      MaterialBanner(
                        content: const Text(contentText),
                        leading: const Icon(Icons.umbrella),
                        actions: <Widget>[
                          TextButton(child: const Text('Action'), onPressed: () {}),
                        ],
                      ),
                    );
                  },
                  behavior: HitTestBehavior.opaque,
                  child: const SizedBox(height: 100.0, width: 100.0),
                );
              },
            ),
          ),
        ),
      );
      await tester.tap(find.byKey(tapTarget));
      await tester.pumpAndSettle();

      final Material material = _getMaterialFromText(tester, contentText);
      expect(material.color, theme.colorScheme.surfaceContainerLow);
      expect(material.surfaceTintColor, Colors.transparent);
      expect(material.shadowColor, null);
      expect(material.elevation, 0.0);

      final RenderParagraph content = _getTextRenderObjectFromDialog(tester, contentText);
      expect(content.text.style, localizedTheme.textTheme.bodyMedium);

      final Offset rowTopLeft = tester.getTopLeft(find.byType(Row));
      final Offset materialTopLeft = tester.getTopLeft(_materialFinder());
      final Offset leadingTopLeft = tester.getTopLeft(find.byIcon(Icons.umbrella));
      expect(rowTopLeft.dy - materialTopLeft.dy, 2.0); // Default single line top padding.
      expect(rowTopLeft.dx - materialTopLeft.dx, 16.0); // Default single line start padding.
      expect(leadingTopLeft.dy - materialTopLeft.dy, 16); // Default leading padding.
      expect(leadingTopLeft.dx - materialTopLeft.dx, 16); // Default leading padding.

      final Divider divider = tester.widget<Divider>(find.byType(Divider));
      expect(divider.color, theme.colorScheme.outlineVariant);
    },
  );

  testWidgets('MaterialBanner uses values from MaterialBannerThemeData', (
    WidgetTester tester,
  ) async {
    final MaterialBannerThemeData bannerTheme = _bannerTheme();
    const String contentText = 'Content';
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(bannerTheme: bannerTheme),
        home: Scaffold(
          body: MaterialBanner(
            leading: const Icon(Icons.ac_unit),
            content: const Text(contentText),
            actions: <Widget>[TextButton(child: const Text('Action'), onPressed: () {})],
          ),
        ),
      ),
    );

    final Material material = _getMaterialFromText(tester, contentText);
    expect(material.color, bannerTheme.backgroundColor);
    expect(material.surfaceTintColor, bannerTheme.surfaceTintColor);
    expect(material.shadowColor, bannerTheme.shadowColor);
    expect(material.elevation, bannerTheme.elevation);

    final RenderParagraph content = _getTextRenderObjectFromDialog(tester, contentText);
    expect(content.text.style, bannerTheme.contentTextStyle);

    final Offset contentTopLeft = tester.getTopLeft(_textFinder(contentText));
    final Offset materialTopLeft = tester.getTopLeft(_materialFinder());
    final Offset leadingTopLeft = tester.getTopLeft(find.byIcon(Icons.ac_unit));
    expect(contentTopLeft.dy - materialTopLeft.dy, 24);
    expect(contentTopLeft.dx - materialTopLeft.dx, 41);
    expect(leadingTopLeft.dy - materialTopLeft.dy, 19);
    expect(leadingTopLeft.dx - materialTopLeft.dx, 11);

    expect(find.byType(Divider), findsNothing);
  });

  testWidgets(
    'MaterialBanner uses values from MaterialBannerThemeData when presented by ScaffoldMessenger',
    (WidgetTester tester) async {
      final MaterialBannerThemeData bannerTheme = _bannerTheme();
      const String contentText = 'Content';
      const Key tapTarget = Key('tap-target');
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(bannerTheme: bannerTheme),
          home: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  key: tapTarget,
                  onTap: () {
                    ScaffoldMessenger.of(context).showMaterialBanner(
                      MaterialBanner(
                        leading: const Icon(Icons.ac_unit),
                        content: const Text(contentText),
                        actions: <Widget>[
                          TextButton(child: const Text('Action'), onPressed: () {}),
                        ],
                      ),
                    );
                  },
                  behavior: HitTestBehavior.opaque,
                  child: const SizedBox(height: 100.0, width: 100.0),
                );
              },
            ),
          ),
        ),
      );
      await tester.tap(find.byKey(tapTarget));
      await tester.pumpAndSettle();

      final Material material = _getMaterialFromText(tester, contentText);
      expect(material.color, bannerTheme.backgroundColor);
      expect(material.surfaceTintColor, bannerTheme.surfaceTintColor);
      expect(material.shadowColor, bannerTheme.shadowColor);
      expect(material.elevation, bannerTheme.elevation);

      final RenderParagraph content = _getTextRenderObjectFromDialog(tester, contentText);
      expect(content.text.style, bannerTheme.contentTextStyle);

      final Offset contentTopLeft = tester.getTopLeft(_textFinder(contentText));
      final Offset materialTopLeft = tester.getTopLeft(_materialFinder());
      final Offset leadingTopLeft = tester.getTopLeft(find.byIcon(Icons.ac_unit));
      expect(contentTopLeft.dy - materialTopLeft.dy, 24);
      expect(contentTopLeft.dx - materialTopLeft.dx, 41);
      expect(leadingTopLeft.dy - materialTopLeft.dy, 19);
      expect(leadingTopLeft.dx - materialTopLeft.dx, 11);

      expect(find.byType(Divider), findsNothing);
    },
  );

  testWidgets('MaterialBanner widget properties take priority over theme', (
    WidgetTester tester,
  ) async {
    const Color backgroundColor = Colors.purple;
    const Color surfaceTintColor = Colors.red;
    const Color shadowColor = Colors.orange;
    const TextStyle textStyle = TextStyle(color: Colors.green);
    final MaterialBannerThemeData bannerTheme = _bannerTheme();
    const String contentText = 'Content';

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(bannerTheme: bannerTheme),
        home: Scaffold(
          body: MaterialBanner(
            backgroundColor: backgroundColor,
            surfaceTintColor: surfaceTintColor,
            shadowColor: shadowColor,
            elevation: 6.0,
            leading: const Icon(Icons.ac_unit),
            contentTextStyle: textStyle,
            content: const Text(contentText),
            padding: const EdgeInsets.all(10),
            leadingPadding: const EdgeInsets.all(12),
            actions: <Widget>[TextButton(child: const Text('Action'), onPressed: () {})],
          ),
        ),
      ),
    );

    final Material material = _getMaterialFromText(tester, contentText);
    expect(material.color, backgroundColor);
    expect(material.surfaceTintColor, surfaceTintColor);
    expect(material.shadowColor, shadowColor);
    expect(material.elevation, 6.0);

    final RenderParagraph content = _getTextRenderObjectFromDialog(tester, contentText);
    expect(content.text.style, textStyle);

    final Offset contentTopLeft = tester.getTopLeft(_textFinder(contentText));
    final Offset materialTopLeft = tester.getTopLeft(_materialFinder());
    final Offset leadingTopLeft = tester.getTopLeft(find.byIcon(Icons.ac_unit));
    expect(contentTopLeft.dy - materialTopLeft.dy, 29);
    expect(contentTopLeft.dx - materialTopLeft.dx, 58);
    expect(leadingTopLeft.dy - materialTopLeft.dy, 24);
    expect(leadingTopLeft.dx - materialTopLeft.dx, 22);

    expect(find.byType(Divider), findsNothing);
  });

  testWidgets(
    'MaterialBanner widget properties take priority over theme when presented by ScaffoldMessenger',
    (WidgetTester tester) async {
      const Color backgroundColor = Colors.purple;
      const double elevation = 6.0;
      const TextStyle textStyle = TextStyle(color: Colors.green);
      final MaterialBannerThemeData bannerTheme = _bannerTheme();
      const String contentText = 'Content';
      const Key tapTarget = Key('tap-target');

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(bannerTheme: bannerTheme),
          home: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  key: tapTarget,
                  onTap: () {
                    ScaffoldMessenger.of(context).showMaterialBanner(
                      MaterialBanner(
                        backgroundColor: backgroundColor,
                        elevation: elevation,
                        leading: const Icon(Icons.ac_unit),
                        contentTextStyle: textStyle,
                        content: const Text(contentText),
                        padding: const EdgeInsets.all(10),
                        leadingPadding: const EdgeInsets.all(12),
                        actions: <Widget>[
                          TextButton(child: const Text('Action'), onPressed: () {}),
                        ],
                      ),
                    );
                  },
                  behavior: HitTestBehavior.opaque,
                  child: const SizedBox(height: 100.0, width: 100.0),
                );
              },
            ),
          ),
        ),
      );
      await tester.tap(find.byKey(tapTarget));
      await tester.pumpAndSettle();

      final Material material = _getMaterialFromText(tester, contentText);
      expect(material.color, backgroundColor);
      expect(material.elevation, elevation);

      final RenderParagraph content = _getTextRenderObjectFromDialog(tester, contentText);
      expect(content.text.style, textStyle);

      final Offset contentTopLeft = tester.getTopLeft(_textFinder(contentText));
      final Offset materialTopLeft = tester.getTopLeft(_materialFinder());
      final Offset leadingTopLeft = tester.getTopLeft(find.byIcon(Icons.ac_unit));
      expect(contentTopLeft.dy - materialTopLeft.dy, 29);
      expect(contentTopLeft.dx - materialTopLeft.dx, 58);
      expect(leadingTopLeft.dy - materialTopLeft.dy, 24);
      expect(leadingTopLeft.dx - materialTopLeft.dx, 22);

      expect(find.byType(Divider), findsNothing);
    },
  );

  testWidgets('MaterialBanner uses color scheme when necessary', (WidgetTester tester) async {
    final ColorScheme colorScheme = const ColorScheme.light().copyWith(surface: Colors.purple);
    const String contentText = 'Content';
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(colorScheme: colorScheme),
        home: Scaffold(
          body: MaterialBanner(
            content: const Text(contentText),
            actions: <Widget>[TextButton(child: const Text('Action'), onPressed: () {})],
          ),
        ),
      ),
    );

    final Material material = _getMaterialFromText(tester, contentText);
    expect(material.color, colorScheme.surfaceContainerLow);
  });

  testWidgets(
    'MaterialBanner uses color scheme when necessary when presented by ScaffoldMessenger',
    (WidgetTester tester) async {
      final ColorScheme colorScheme = const ColorScheme.light().copyWith(surface: Colors.purple);
      const String contentText = 'Content';
      const Key tapTarget = Key('tap-target');
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(colorScheme: colorScheme),
          home: Scaffold(
            body: Builder(
              builder: (BuildContext context) {
                return GestureDetector(
                  key: tapTarget,
                  onTap: () {
                    ScaffoldMessenger.of(context).showMaterialBanner(
                      MaterialBanner(
                        content: const Text(contentText),
                        actions: <Widget>[
                          TextButton(child: const Text('Action'), onPressed: () {}),
                        ],
                      ),
                    );
                  },
                  behavior: HitTestBehavior.opaque,
                  child: const SizedBox(height: 100.0, width: 100.0),
                );
              },
            ),
          ),
        ),
      );
      await tester.tap(find.byKey(tapTarget));
      await tester.pumpAndSettle();

      final Material material = _getMaterialFromText(tester, contentText);
      expect(material.color, colorScheme.surfaceContainerLow);
    },
  );

  group('Material 2', () {
    // These tests are only relevant for Material 2. Once Material 2
    // support is deprecated and the APIs are removed, these tests
    // can be deleted.

    testWidgets('Material2 - Passing no MaterialBannerThemeData returns defaults', (
      WidgetTester tester,
    ) async {
      const String contentText = 'Content';

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Scaffold(
            body: MaterialBanner(
              content: const Text(contentText),
              leading: const Icon(Icons.umbrella),
              actions: <Widget>[TextButton(child: const Text('Action'), onPressed: () {})],
            ),
          ),
        ),
      );

      final Material material = _getMaterialFromText(tester, contentText);
      expect(material.color, const Color(0xffffffff));
      expect(material.surfaceTintColor, null);
      expect(material.shadowColor, null);
      expect(material.elevation, 0.0);

      final RenderParagraph content = _getTextRenderObjectFromDialog(tester, contentText);
      // Default value for ThemeData.typography is Typography.material2014()
      expect(
        content.text.style,
        Typography.material2014().englishLike.bodyMedium!.merge(
          Typography.material2014().black.bodyMedium,
        ),
      );

      final Offset rowTopLeft = tester.getTopLeft(find.byType(Row));
      final Offset materialTopLeft = tester.getTopLeft(_materialFinder());
      final Offset leadingTopLeft = tester.getTopLeft(find.byIcon(Icons.umbrella));
      expect(rowTopLeft.dy - materialTopLeft.dy, 2.0); // Default single line top padding.
      expect(rowTopLeft.dx - materialTopLeft.dx, 16.0); // Default single line start padding.
      expect(leadingTopLeft.dy - materialTopLeft.dy, 16); // Default leading padding.
      expect(leadingTopLeft.dx - materialTopLeft.dx, 16); // Default leading padding.

      final Divider divider = tester.widget<Divider>(find.byType(Divider));
      expect(divider.color, null);
    });

    testWidgets(
      'Material2 - Passing no MaterialBannerThemeData returns defaults when presented by ScaffoldMessenger',
      (WidgetTester tester) async {
        const String contentText = 'Content';
        const Key tapTarget = Key('tap-target');

        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(useMaterial3: false),
            home: Scaffold(
              body: Builder(
                builder: (BuildContext context) {
                  return GestureDetector(
                    key: tapTarget,
                    onTap: () {
                      ScaffoldMessenger.of(context).showMaterialBanner(
                        MaterialBanner(
                          content: const Text(contentText),
                          leading: const Icon(Icons.umbrella),
                          actions: <Widget>[
                            TextButton(child: const Text('Action'), onPressed: () {}),
                          ],
                        ),
                      );
                    },
                    behavior: HitTestBehavior.opaque,
                    child: const SizedBox(height: 100.0, width: 100.0),
                  );
                },
              ),
            ),
          ),
        );
        await tester.tap(find.byKey(tapTarget));
        await tester.pumpAndSettle();

        final Material material = _getMaterialFromText(tester, contentText);
        expect(material.color, const Color(0xffffffff));
        expect(material.surfaceTintColor, null);
        expect(material.shadowColor, null);
        expect(material.elevation, 0.0);

        final RenderParagraph content = _getTextRenderObjectFromDialog(tester, contentText);
        // Default value for ThemeData.typography is Typography.material2014()
        expect(
          content.text.style,
          Typography.material2014().englishLike.bodyMedium!.merge(
            Typography.material2014().black.bodyMedium,
          ),
        );

        final Offset rowTopLeft = tester.getTopLeft(find.byType(Row));
        final Offset materialTopLeft = tester.getTopLeft(_materialFinder());
        final Offset leadingTopLeft = tester.getTopLeft(find.byIcon(Icons.umbrella));
        expect(rowTopLeft.dy - materialTopLeft.dy, 2.0); // Default single line top padding.
        expect(rowTopLeft.dx - materialTopLeft.dx, 16.0); // Default single line start padding.
        expect(leadingTopLeft.dy - materialTopLeft.dy, 16); // Default leading padding.
        expect(leadingTopLeft.dx - materialTopLeft.dx, 16); // Default leading padding.

        final Divider divider = tester.widget<Divider>(find.byType(Divider));
        expect(divider.color, null);
      },
    );
  });
}

MaterialBannerThemeData _bannerTheme() {
  return const MaterialBannerThemeData(
    backgroundColor: Colors.orange,
    surfaceTintColor: Colors.yellow,
    shadowColor: Colors.red,
    dividerColor: Colors.green,
    contentTextStyle: TextStyle(color: Colors.pink),
    elevation: 4.0,
    padding: EdgeInsets.all(5),
    leadingPadding: EdgeInsets.all(6),
  );
}

Material _getMaterialFromText(WidgetTester tester, String text) {
  return tester.widget<Material>(find.widgetWithText(Material, text).first);
}

Finder _materialFinder() {
  return find.descendant(of: find.byType(MaterialBanner), matching: find.byType(Material)).first;
}

RenderParagraph _getTextRenderObjectFromDialog(WidgetTester tester, String text) {
  return tester.element<StatelessElement>(_textFinder(text)).renderObject! as RenderParagraph;
}

Finder _textFinder(String text) {
  return find.descendant(of: find.byType(MaterialBanner), matching: find.text(text));
}
