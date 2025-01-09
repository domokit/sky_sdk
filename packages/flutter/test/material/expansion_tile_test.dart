// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/foundation.dart';
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
    return const Icon(Icons.expand_more);
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
  const Color dividerColor = Color(0x1f333333);
  const Color foregroundColor = Colors.blueAccent;
  const Color unselectedWidgetColor = Colors.black54;
  const Color headerColor = Colors.black45;

  Material getMaterial(WidgetTester tester) {
    return tester.widget<Material>(
      find.descendant(of: find.byType(ExpansionTile), matching: find.byType(Material)),
    );
  }

  testWidgets(
    'ExpansionTile initial state',
    (WidgetTester tester) async {
      final Key topKey = UniqueKey();
      final Key tileKey = UniqueKey();
      const Key expandedKey = PageStorageKey<String>('expanded');
      const Key collapsedKey = PageStorageKey<String>('collapsed');
      const Key defaultKey = PageStorageKey<String>('default');

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(dividerColor: dividerColor),
          home: Material(
            child: SingleChildScrollView(
              child: Column(
                children: <Widget>[
                  ListTile(title: const Text('Top'), key: topKey),
                  ExpansionTile(
                    key: expandedKey,
                    initiallyExpanded: true,
                    title: const Text('Expanded'),
                    backgroundColor: Colors.red,
                    children: <Widget>[ListTile(key: tileKey, title: const Text('0'))],
                  ),
                  ExpansionTile(
                    key: collapsedKey,
                    title: const Text('Collapsed'),
                    children: <Widget>[ListTile(key: tileKey, title: const Text('0'))],
                  ),
                  const ExpansionTile(
                    key: defaultKey,
                    title: Text('Default'),
                    children: <Widget>[ListTile(title: Text('0'))],
                  ),
                ],
              ),
            ),
          ),
        ),
      );

      double getHeight(Key key) => tester.getSize(find.byKey(key)).height;
      DecoratedBox getDecoratedBox(Key key) => tester.firstWidget(
        find.descendant(of: find.byKey(key), matching: find.byType(DecoratedBox)),
      );

      expect(getHeight(topKey), getHeight(expandedKey) - getHeight(tileKey) - 2.0);
      expect(getHeight(topKey), getHeight(collapsedKey) - 2.0);
      expect(getHeight(topKey), getHeight(defaultKey) - 2.0);

      ShapeDecoration expandedContainerDecoration =
          getDecoratedBox(expandedKey).decoration as ShapeDecoration;
      expect(expandedContainerDecoration.color, Colors.red);
      expect((expandedContainerDecoration.shape as Border).top.color, dividerColor);
      expect((expandedContainerDecoration.shape as Border).bottom.color, dividerColor);

      ShapeDecoration collapsedContainerDecoration =
          getDecoratedBox(collapsedKey).decoration as ShapeDecoration;
      expect(collapsedContainerDecoration.color, Colors.transparent);
      expect((collapsedContainerDecoration.shape as Border).top.color, Colors.transparent);
      expect((collapsedContainerDecoration.shape as Border).bottom.color, Colors.transparent);

      await tester.tap(find.text('Expanded'));
      await tester.tap(find.text('Collapsed'));
      await tester.tap(find.text('Default'));

      await tester.pump();

      // Pump to the middle of the animation for expansion.
      await tester.pump(const Duration(milliseconds: 100));
      final ShapeDecoration collapsingContainerDecoration =
          getDecoratedBox(collapsedKey).decoration as ShapeDecoration;
      expect(collapsingContainerDecoration.color, Colors.transparent);
      expect(
        (collapsingContainerDecoration.shape as Border).top.color,
        isSameColorAs(const Color(0x15222222)),
      );
      expect(
        (collapsingContainerDecoration.shape as Border).bottom.color,
        isSameColorAs(const Color(0x15222222)),
      );

      // Pump all the way to the end now.
      await tester.pump(const Duration(seconds: 1));

      expect(getHeight(topKey), getHeight(expandedKey) - 2.0);
      expect(getHeight(topKey), getHeight(collapsedKey) - getHeight(tileKey) - 2.0);
      expect(getHeight(topKey), getHeight(defaultKey) - getHeight(tileKey) - 2.0);

      // Expanded should be collapsed now.
      expandedContainerDecoration = getDecoratedBox(expandedKey).decoration as ShapeDecoration;
      expect(expandedContainerDecoration.color, Colors.transparent);
      expect((expandedContainerDecoration.shape as Border).top.color, Colors.transparent);
      expect((expandedContainerDecoration.shape as Border).bottom.color, Colors.transparent);

      // Collapsed should be expanded now.
      collapsedContainerDecoration = getDecoratedBox(collapsedKey).decoration as ShapeDecoration;
      expect(collapsedContainerDecoration.color, Colors.transparent);
      expect((collapsedContainerDecoration.shape as Border).top.color, dividerColor);
      expect((collapsedContainerDecoration.shape as Border).bottom.color, dividerColor);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'ExpansionTile Theme dependencies',
    (WidgetTester tester) async {
      final Key expandedTitleKey = UniqueKey();
      final Key collapsedTitleKey = UniqueKey();
      final Key expandedIconKey = UniqueKey();
      final Key collapsedIconKey = UniqueKey();

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            useMaterial3: false,
            colorScheme: ColorScheme.fromSwatch().copyWith(primary: foregroundColor),
            unselectedWidgetColor: unselectedWidgetColor,
            textTheme: const TextTheme(titleMedium: TextStyle(color: headerColor)),
          ),
          home: Material(
            child: SingleChildScrollView(
              child: Column(
                children: <Widget>[
                  const ListTile(title: Text('Top')),
                  ExpansionTile(
                    initiallyExpanded: true,
                    title: TestText('Expanded', key: expandedTitleKey),
                    backgroundColor: Colors.red,
                    trailing: TestIcon(key: expandedIconKey),
                    children: const <Widget>[ListTile(title: Text('0'))],
                  ),
                  ExpansionTile(
                    title: TestText('Collapsed', key: collapsedTitleKey),
                    trailing: TestIcon(key: collapsedIconKey),
                    children: const <Widget>[ListTile(title: Text('0'))],
                  ),
                ],
              ),
            ),
          ),
        ),
      );

      Color iconColor(Key key) => tester.state<TestIconState>(find.byKey(key)).iconTheme.color!;
      Color textColor(Key key) => tester.state<TestTextState>(find.byKey(key)).textStyle.color!;

      expect(textColor(expandedTitleKey), foregroundColor);
      expect(textColor(collapsedTitleKey), headerColor);
      expect(iconColor(expandedIconKey), foregroundColor);
      expect(iconColor(collapsedIconKey), unselectedWidgetColor);

      // Tap both tiles to change their state: collapse and extend respectively
      await tester.tap(find.text('Expanded'));
      await tester.tap(find.text('Collapsed'));
      await tester.pump();
      await tester.pump(const Duration(seconds: 1));
      await tester.pump(const Duration(seconds: 1));

      expect(textColor(expandedTitleKey), headerColor);
      expect(textColor(collapsedTitleKey), foregroundColor);
      expect(iconColor(expandedIconKey), unselectedWidgetColor);
      expect(iconColor(collapsedIconKey), foregroundColor);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('ExpansionTile subtitle', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: ExpansionTile(
            title: Text('Title'),
            subtitle: Text('Subtitle'),
            children: <Widget>[ListTile(title: Text('0'))],
          ),
        ),
      ),
    );

    expect(find.text('Subtitle'), findsOneWidget);
  });

  testWidgets('ExpansionTile maintainState', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(platform: TargetPlatform.iOS, dividerColor: dividerColor),
        home: const Material(
          child: SingleChildScrollView(
            child: Column(
              children: <Widget>[
                ExpansionTile(
                  title: Text('Tile 1'),
                  maintainState: true,
                  children: <Widget>[Text('Maintaining State')],
                ),
                ExpansionTile(title: Text('Title 2'), children: <Widget>[Text('Discarding State')]),
              ],
            ),
          ),
        ),
      ),
    );

    // This text should be offstage while ExpansionTile collapsed
    expect(find.text('Maintaining State', skipOffstage: false), findsOneWidget);
    expect(find.text('Maintaining State'), findsNothing);
    // This text shouldn't be there while ExpansionTile collapsed
    expect(find.text('Discarding State'), findsNothing);
  });

  testWidgets('ExpansionTile padding test', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: Center(
            child: ExpansionTile(
              title: Text('Hello'),
              tilePadding: EdgeInsets.fromLTRB(8, 12, 4, 10),
            ),
          ),
        ),
      ),
    );

    final Rect titleRect = tester.getRect(find.text('Hello'));
    final Rect trailingRect = tester.getRect(find.byIcon(Icons.expand_more));
    final Rect listTileRect = tester.getRect(find.byType(ListTile));
    final Rect tallerWidget = titleRect.height > trailingRect.height ? titleRect : trailingRect;

    // Check the positions of title and trailing Widgets, after padding is applied.
    expect(listTileRect.left, titleRect.left - 8);
    expect(listTileRect.right, trailingRect.right + 4);

    // Calculate the remaining height of ListTile from the default height.
    final double remainingHeight = 56 - tallerWidget.height;
    expect(listTileRect.top, tallerWidget.top - remainingHeight / 2 - 12);
    expect(listTileRect.bottom, tallerWidget.bottom + remainingHeight / 2 + 10);
  });

  testWidgets('ExpansionTile expandedAlignment test', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: Center(
            child: ExpansionTile(
              title: Text('title'),
              expandedAlignment: Alignment.centerLeft,
              children: <Widget>[
                SizedBox(height: 100, width: 100),
                SizedBox(height: 100, width: 80),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('title'));
    await tester.pumpAndSettle();

    final Rect columnRect = tester.getRect(find.byType(Column).last);

    // The expandedAlignment is used to define the alignment of the Column widget in
    // expanded tile, not the alignment of the children inside the Column.
    expect(columnRect.left, 0.0);
    // The width of the Column is the width of the largest child. The largest width
    // being 100.0, the offset of the right edge of Column from X-axis should be 100.0.
    expect(columnRect.right, 100.0);
  });

  testWidgets('ExpansionTile expandedCrossAxisAlignment test', (WidgetTester tester) async {
    const Key child0Key = Key('child0');
    const Key child1Key = Key('child1');

    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: Center(
            child: ExpansionTile(
              title: Text('title'),
              // Set the column's alignment to Alignment.centerRight to test CrossAxisAlignment
              // of children widgets. This helps distinguish the effect of expandedAlignment
              // and expandedCrossAxisAlignment later in the test.
              expandedAlignment: Alignment.centerRight,
              expandedCrossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                SizedBox(height: 100, width: 100, key: child0Key),
                SizedBox(height: 100, width: 80, key: child1Key),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('title'));
    await tester.pumpAndSettle();

    final Rect columnRect = tester.getRect(find.byType(Column).last);
    final Rect child0Rect = tester.getRect(find.byKey(child0Key));
    final Rect child1Rect = tester.getRect(find.byKey(child1Key));

    // Since expandedAlignment is set to Alignment.centerRight, the column of children
    // should be aligned to the center right of the expanded tile. This provides confirmation
    // that the expandedCrossAxisAlignment.start is 700.0, where columnRect.left is.
    expect(columnRect.right, 800.0);
    // The width of the Column is the width of the largest child. The largest width
    // being 100.0, the offset of the left edge of Column from X-axis should be 700.0.
    expect(columnRect.left, 700.0);

    // Considering the value of expandedCrossAxisAlignment is CrossAxisAlignment.start,
    // the offset of the left edge of both the children from X-axis should be 700.0.
    expect(child0Rect.left, 700.0);
    expect(child1Rect.left, 700.0);
  });

  testWidgets('CrossAxisAlignment.baseline is not allowed', (WidgetTester tester) async {
    expect(
      () {
        MaterialApp(
          home: Material(
            child: ExpansionTile(
              initiallyExpanded: true,
              title: const Text('title'),
              expandedCrossAxisAlignment: CrossAxisAlignment.baseline,
            ),
          ),
        );
      },
      throwsA(
        isA<AssertionError>().having(
          (AssertionError error) => error.toString(),
          '.toString()',
          contains(
            'CrossAxisAlignment.baseline is not supported since the expanded'
            ' children are aligned in a column, not a row. Try to use another constant.',
          ),
        ),
      ),
    );
  });

  testWidgets('expandedCrossAxisAlignment and expandedAlignment default values', (
    WidgetTester tester,
  ) async {
    const Key child1Key = Key('child1');

    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: Center(
            child: ExpansionTile(
              title: Text('title'),
              children: <Widget>[
                SizedBox(height: 100, width: 100),
                SizedBox(height: 100, width: 80, key: child1Key),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('title'));
    await tester.pumpAndSettle();

    final Rect columnRect = tester.getRect(find.byType(Column).last);
    final Rect child1Rect = tester.getRect(find.byKey(child1Key));

    // The default viewport size is Size(800, 600).
    // By default the value of extendedAlignment is Alignment.center, hence the offset
    // of left and right edges from x axis should be equal.
    expect(columnRect.left, 800 - columnRect.right);

    // By default the value of extendedCrossAxisAlignment is CrossAxisAlignment.center, hence
    // the offset of left and right edges from Column should be equal.
    expect(child1Rect.left - columnRect.left, columnRect.right - child1Rect.right);
  });

  testWidgets('childrenPadding default value', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: Center(
            child: ExpansionTile(
              title: Text('title'),
              children: <Widget>[SizedBox(height: 100, width: 100)],
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('title'));
    await tester.pumpAndSettle();

    final Rect columnRect = tester.getRect(find.byType(Column).last);
    final Rect paddingRect = tester.getRect(find.byType(Padding).last);

    // By default, the value of childrenPadding is EdgeInsets.zero, hence offset
    // of all the edges from x-axis and y-axis should be equal for Padding and Column.
    expect(columnRect.top, paddingRect.top);
    expect(columnRect.left, paddingRect.left);
    expect(columnRect.right, paddingRect.right);
    expect(columnRect.bottom, paddingRect.bottom);
  });

  testWidgets('ExpansionTile childrenPadding test', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: Center(
            child: ExpansionTile(
              title: Text('title'),
              childrenPadding: EdgeInsets.fromLTRB(10, 8, 12, 4),
              children: <Widget>[SizedBox(height: 100, width: 100)],
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('title'));
    await tester.pumpAndSettle();

    final Rect columnRect = tester.getRect(find.byType(Column).last);
    final Rect paddingRect = tester.getRect(find.byType(Padding).last);

    // Check the offset of all the edges from x-axis and y-axis after childrenPadding
    // is applied.
    expect(columnRect.left, paddingRect.left + 10);
    expect(columnRect.top, paddingRect.top + 8);
    expect(columnRect.right, paddingRect.right - 12);
    expect(columnRect.bottom, paddingRect.bottom - 4);
  });

  testWidgets('ExpansionTile.collapsedBackgroundColor', (WidgetTester tester) async {
    const Key expansionTileKey = Key('expansionTileKey');
    const Color backgroundColor = Colors.red;
    const Color collapsedBackgroundColor = Colors.brown;

    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: ExpansionTile(
            key: expansionTileKey,
            title: Text('Title'),
            backgroundColor: backgroundColor,
            collapsedBackgroundColor: collapsedBackgroundColor,
            children: <Widget>[SizedBox(height: 100, width: 100)],
          ),
        ),
      ),
    );

    ShapeDecoration shapeDecoration =
        tester
                .firstWidget<DecoratedBox>(
                  find.descendant(
                    of: find.byKey(expansionTileKey),
                    matching: find.byType(DecoratedBox),
                  ),
                )
                .decoration
            as ShapeDecoration;

    expect(shapeDecoration.color, collapsedBackgroundColor);

    await tester.tap(find.text('Title'));
    await tester.pumpAndSettle();

    shapeDecoration =
        tester
                .firstWidget<DecoratedBox>(
                  find.descendant(
                    of: find.byKey(expansionTileKey),
                    matching: find.byType(DecoratedBox),
                  ),
                )
                .decoration
            as ShapeDecoration;

    expect(shapeDecoration.color, backgroundColor);
  });

  testWidgets('ExpansionTile default iconColor, textColor', (WidgetTester tester) async {
    final ThemeData theme = ThemeData(useMaterial3: true);

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Material(
          child: ExpansionTile(
            title: TestText('title'),
            trailing: TestIcon(),
            children: <Widget>[SizedBox(height: 100, width: 100)],
          ),
        ),
      ),
    );

    Color getIconColor() => tester.state<TestIconState>(find.byType(TestIcon)).iconTheme.color!;
    Color getTextColor() => tester.state<TestTextState>(find.byType(TestText)).textStyle.color!;

    expect(getIconColor(), theme.colorScheme.onSurfaceVariant);
    expect(getTextColor(), theme.colorScheme.onSurface);

    await tester.tap(find.text('title'));
    await tester.pumpAndSettle();

    expect(getIconColor(), theme.colorScheme.primary);
    expect(getTextColor(), theme.colorScheme.onSurface);
  });

  testWidgets('ExpansionTile iconColor, textColor', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/pull/78281

    const Color iconColor = Color(0xff00ff00);
    const Color collapsedIconColor = Color(0xff0000ff);
    const Color textColor = Color(0xff00ffff);
    const Color collapsedTextColor = Color(0xffff00ff);

    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: ExpansionTile(
            iconColor: iconColor,
            collapsedIconColor: collapsedIconColor,
            textColor: textColor,
            collapsedTextColor: collapsedTextColor,
            title: TestText('title'),
            trailing: TestIcon(),
            children: <Widget>[SizedBox(height: 100, width: 100)],
          ),
        ),
      ),
    );

    Color getIconColor() => tester.state<TestIconState>(find.byType(TestIcon)).iconTheme.color!;
    Color getTextColor() => tester.state<TestTextState>(find.byType(TestText)).textStyle.color!;

    expect(getIconColor(), collapsedIconColor);
    expect(getTextColor(), collapsedTextColor);

    await tester.tap(find.text('title'));
    await tester.pumpAndSettle();

    expect(getIconColor(), iconColor);
    expect(getTextColor(), textColor);
  });

  testWidgets('ExpansionTile Border', (WidgetTester tester) async {
    const Key expansionTileKey = PageStorageKey<String>('expansionTile');

    const Border collapsedShape = Border(
      top: BorderSide(color: Colors.blue),
      bottom: BorderSide(color: Colors.green),
    );
    final Border shape = Border.all(color: Colors.red);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: ExpansionTile(
            key: expansionTileKey,
            title: const Text('ExpansionTile'),
            collapsedShape: collapsedShape,
            shape: shape,
            children: const <Widget>[ListTile(title: Text('0'))],
          ),
        ),
      ),
    );

    // When a custom shape is provided, ExpansionTile will use the
    // Material widget to draw the shape and background color
    // instead of a Container.
    Material material = getMaterial(tester);
    // ExpansionTile should be collapsed initially.
    expect(material.shape, collapsedShape);
    expect(material.clipBehavior, Clip.antiAlias);

    await tester.tap(find.text('ExpansionTile'));
    await tester.pumpAndSettle();

    // ExpansionTile should be Expanded now.
    material = getMaterial(tester);
    expect(material.shape, shape);
    expect(material.clipBehavior, Clip.antiAlias);
  });

  testWidgets('ExpansionTile platform controlAffinity test', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: ExpansionTile(title: Text('Title')))),
    );

    final ListTile listTile = tester.widget(find.byType(ListTile));
    expect(listTile.leading, isNull);
    expect(listTile.trailing.runtimeType, RotationTransition);
  });

  testWidgets('ExpansionTile trailing controlAffinity test', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: ExpansionTile(
            title: Text('Title'),
            controlAffinity: ListTileControlAffinity.trailing,
          ),
        ),
      ),
    );

    final ListTile listTile = tester.widget(find.byType(ListTile));
    expect(listTile.leading, isNull);
    expect(listTile.trailing.runtimeType, RotationTransition);
  });

  testWidgets('ExpansionTile leading controlAffinity test', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: ExpansionTile(
            title: Text('Title'),
            controlAffinity: ListTileControlAffinity.leading,
          ),
        ),
      ),
    );

    final ListTile listTile = tester.widget(find.byType(ListTile));
    expect(listTile.leading.runtimeType, RotationTransition);
    expect(listTile.trailing, isNull);
  });

  testWidgets('ExpansionTile override rotating icon test', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: ExpansionTile(
            title: Text('Title'),
            leading: Icon(Icons.info),
            controlAffinity: ListTileControlAffinity.leading,
          ),
        ),
      ),
    );

    final ListTile listTile = tester.widget(find.byType(ListTile));
    expect(listTile.leading.runtimeType, Icon);
    expect(listTile.trailing, isNull);
  });

  testWidgets('Nested ListTile Semantics', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    final SemanticsHandle handle = tester.ensureSemantics();

    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: Column(
            children: <Widget>[
              ExpansionTile(title: Text('First Expansion Tile'), internalAddSemanticForOnTap: true),
              ExpansionTile(
                initiallyExpanded: true,
                title: Text('Second Expansion Tile'),
                internalAddSemanticForOnTap: true,
              ),
            ],
          ),
        ),
      ),
    );

    await tester.pumpAndSettle();

    // Focus the first ExpansionTile.
    tester.binding.focusManager.primaryFocus?.nextFocus();
    await tester.pumpAndSettle();

    // The first list tile is focused.
    expect(
      tester.getSemantics(find.byType(ListTile).first),
      matchesSemantics(
        isButton: true,
        hasTapAction: true,
        hasFocusAction: true,
        hasEnabledState: true,
        isEnabled: true,
        isFocused: true,
        isFocusable: true,
        label: 'First Expansion Tile',
        textDirection: TextDirection.ltr,
      ),
    );

    // The first list tile is not focused.
    expect(
      tester.getSemantics(find.byType(ListTile).last),
      matchesSemantics(
        isButton: true,
        hasTapAction: true,
        hasFocusAction: true,
        hasEnabledState: true,
        isEnabled: true,
        isFocusable: true,
        label: 'Second Expansion Tile',
        textDirection: TextDirection.ltr,
      ),
    );
    handle.dispose();
  });

  testWidgets(
    'ExpansionTile Semantics announcement',
    (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      const DefaultMaterialLocalizations localizations = DefaultMaterialLocalizations();
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: ExpansionTile(
              title: Text('Title'),
              children: <Widget>[SizedBox(height: 100, width: 100)],
            ),
          ),
        ),
      );

      // There is no semantics announcement without tap action.
      expect(tester.takeAnnouncements(), isEmpty);

      // Tap the title to expand ExpansionTile.
      await tester.tap(find.text('Title'));
      await tester.pumpAndSettle();

      // The announcement should be the opposite of the current state.
      // The ExpansionTile is expanded, so the announcement should be
      // "Expanded".
      expect(tester.takeAnnouncements().first.message, localizations.collapsedHint);

      // Tap the title to collapse ExpansionTile.
      await tester.tap(find.text('Title'));
      await tester.pumpAndSettle();

      // The announcement should be the opposite of the current state.
      // The ExpansionTile is collapsed, so the announcement should be
      // "Collapsed".
      expect(tester.takeAnnouncements().first.message, localizations.expandedHint);
      handle.dispose();
    },
    // [intended] https://github.com/flutter/flutter/issues/122101.
    skip: defaultTargetPlatform == TargetPlatform.iOS,
  );

  // This is a regression test for https://github.com/flutter/flutter/issues/132264.
  testWidgets(
    'ExpansionTile Semantics announcement is delayed on iOS',
    (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      const DefaultMaterialLocalizations localizations = DefaultMaterialLocalizations();
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: ExpansionTile(
              title: Text('Title'),
              children: <Widget>[SizedBox(height: 100, width: 100)],
            ),
          ),
        ),
      );

      // There is no semantics announcement without tap action.
      expect(tester.takeAnnouncements(), isEmpty);

      // Tap the title to expand ExpansionTile.
      await tester.tap(find.text('Title'));
      await tester.pump(const Duration(seconds: 1)); // Wait for the announcement to be made.

      expect(tester.takeAnnouncements().first.message, localizations.collapsedHint);

      // Tap the title to collapse ExpansionTile.
      await tester.tap(find.text('Title'));
      await tester.pump(const Duration(seconds: 1)); // Wait for the announcement to be made.

      expect(tester.takeAnnouncements().first.message, localizations.expandedHint);
      handle.dispose();
    },
    variant: TargetPlatformVariant.only(TargetPlatform.iOS),
  );

  testWidgets('Semantics with the onTapHint is an ancestor of ListTile', (
    WidgetTester tester,
  ) async {
    // This is a regression test for https://github.com/flutter/flutter/pull/121624
    final SemanticsHandle handle = tester.ensureSemantics();
    const DefaultMaterialLocalizations localizations = DefaultMaterialLocalizations();

    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: Column(
            children: <Widget>[
              ExpansionTile(title: Text('First Expansion Tile')),
              ExpansionTile(initiallyExpanded: true, title: Text('Second Expansion Tile')),
            ],
          ),
        ),
      ),
    );

    SemanticsNode semantics = tester.getSemantics(
      find.ancestor(of: find.byType(ListTile).first, matching: find.byType(Semantics)).first,
    );
    expect(semantics, isNotNull);
    // The onTapHint is passed to semantics properties's hintOverrides.
    expect(semantics.hintOverrides, isNotNull);
    // The hint should be the opposite of the current state.
    // The first ExpansionTile is collapsed, so the hint should be
    // "double tap to expand".
    expect(semantics.hintOverrides!.onTapHint, localizations.expansionTileCollapsedTapHint);

    semantics = tester.getSemantics(
      find.ancestor(of: find.byType(ListTile).last, matching: find.byType(Semantics)).first,
    );

    expect(semantics, isNotNull);
    // The onTapHint is passed to semantics properties's hintOverrides.
    expect(semantics.hintOverrides, isNotNull);
    // The hint should be the opposite of the current state.
    // The second ExpansionTile is expanded, so the hint should be
    // "double tap to collapse".
    expect(semantics.hintOverrides!.onTapHint, localizations.expansionTileExpandedTapHint);
    handle.dispose();
  });

  testWidgets(
    'Semantics hint for iOS and macOS',
    (WidgetTester tester) async {
      final SemanticsHandle handle = tester.ensureSemantics();
      const DefaultMaterialLocalizations localizations = DefaultMaterialLocalizations();

      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: Column(
              children: <Widget>[
                ExpansionTile(title: Text('First Expansion Tile')),
                ExpansionTile(initiallyExpanded: true, title: Text('Second Expansion Tile')),
              ],
            ),
          ),
        ),
      );

      SemanticsNode semantics = tester.getSemantics(
        find.ancestor(of: find.byType(ListTile).first, matching: find.byType(Semantics)).first,
      );

      expect(semantics, isNotNull);
      expect(
        semantics.hint,
        '${localizations.expandedHint}\n ${localizations.expansionTileCollapsedHint}',
      );

      semantics = tester.getSemantics(
        find.ancestor(of: find.byType(ListTile).last, matching: find.byType(Semantics)).first,
      );

      expect(semantics, isNotNull);
      expect(
        semantics.hint,
        '${localizations.collapsedHint}\n ${localizations.expansionTileExpandedHint}',
      );
      handle.dispose();
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets('Collapsed ExpansionTile properties can be updated with setState', (
    WidgetTester tester,
  ) async {
    const Key expansionTileKey = Key('expansionTileKey');
    ShapeBorder collapsedShape = const RoundedRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(4)),
    );
    Color collapsedTextColor = const Color(0xffffffff);
    Color collapsedBackgroundColor = const Color(0xffff0000);
    Color collapsedIconColor = const Color(0xffffffff);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Column(
                children: <Widget>[
                  ExpansionTile(
                    key: expansionTileKey,
                    collapsedShape: collapsedShape,
                    collapsedTextColor: collapsedTextColor,
                    collapsedBackgroundColor: collapsedBackgroundColor,
                    collapsedIconColor: collapsedIconColor,
                    title: const TestText('title'),
                    trailing: const TestIcon(),
                    children: const <Widget>[SizedBox(height: 100, width: 100)],
                  ),
                  // This button is used to update the ExpansionTile properties.
                  FilledButton(
                    onPressed: () {
                      setState(() {
                        collapsedShape = const RoundedRectangleBorder(
                          borderRadius: BorderRadius.all(Radius.circular(16)),
                        );
                        collapsedTextColor = const Color(0xff000000);
                        collapsedBackgroundColor = const Color(0xffffff00);
                        collapsedIconColor = const Color(0xff000000);
                      });
                    },
                    child: const Text('Update collapsed properties'),
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );

    // When a custom shape is provided, ExpansionTile will use the
    // Material widget to draw the shape and background color
    // instead of a Container.
    Material material = getMaterial(tester);

    // Test initial ExpansionTile properties.
    expect(
      material.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4))),
    );
    expect(material.color, const Color(0xffff0000));
    expect(material.clipBehavior, Clip.antiAlias);
    expect(
      tester.state<TestIconState>(find.byType(TestIcon)).iconTheme.color,
      const Color(0xffffffff),
    );
    expect(
      tester.state<TestTextState>(find.byType(TestText)).textStyle.color,
      const Color(0xffffffff),
    );

    // Tap the button to update the ExpansionTile properties.
    await tester.tap(find.text('Update collapsed properties'));
    await tester.pumpAndSettle();

    material = getMaterial(tester);

    // Test updated ExpansionTile properties.
    expect(
      material.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(16))),
    );
    expect(material.color, const Color(0xffffff00));
    expect(material.clipBehavior, Clip.antiAlias);
    expect(
      tester.state<TestIconState>(find.byType(TestIcon)).iconTheme.color,
      const Color(0xff000000),
    );
    expect(
      tester.state<TestTextState>(find.byType(TestText)).textStyle.color,
      const Color(0xff000000),
    );
  });

  testWidgets('Expanded ExpansionTile properties can be updated with setState', (
    WidgetTester tester,
  ) async {
    const Key expansionTileKey = Key('expansionTileKey');
    ShapeBorder shape = const RoundedRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(12)),
    );
    Color textColor = const Color(0xff00ffff);
    Color backgroundColor = const Color(0xff0000ff);
    Color iconColor = const Color(0xff00ffff);

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Column(
                children: <Widget>[
                  ExpansionTile(
                    key: expansionTileKey,
                    shape: shape,
                    textColor: textColor,
                    backgroundColor: backgroundColor,
                    iconColor: iconColor,
                    title: const TestText('title'),
                    trailing: const TestIcon(),
                    children: const <Widget>[SizedBox(height: 100, width: 100)],
                  ),
                  // This button is used to update the ExpansionTile properties.
                  FilledButton(
                    onPressed: () {
                      setState(() {
                        shape = const RoundedRectangleBorder(
                          borderRadius: BorderRadius.all(Radius.circular(6)),
                        );
                        textColor = const Color(0xffffffff);
                        backgroundColor = const Color(0xff123456);
                        iconColor = const Color(0xffffffff);
                      });
                    },
                    child: const Text('Update collapsed properties'),
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );

    // Tap to expand the ExpansionTile.
    await tester.tap(find.text('title'));
    await tester.pumpAndSettle();

    // When a custom shape is provided, ExpansionTile will use the
    // Material widget to draw the shape and background color
    // instead of a Container.
    Material material = getMaterial(tester);

    // Test initial ExpansionTile properties.
    expect(
      material.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(12))),
    );
    expect(material.color, const Color(0xff0000ff));
    expect(material.clipBehavior, Clip.antiAlias);
    expect(
      tester.state<TestIconState>(find.byType(TestIcon)).iconTheme.color,
      const Color(0xff00ffff),
    );
    expect(
      tester.state<TestTextState>(find.byType(TestText)).textStyle.color,
      const Color(0xff00ffff),
    );

    // Tap the button to update the ExpansionTile properties.
    await tester.tap(find.text('Update collapsed properties'));
    await tester.pumpAndSettle();

    material = getMaterial(tester);
    iconColor = tester.state<TestIconState>(find.byType(TestIcon)).iconTheme.color!;
    textColor = tester.state<TestTextState>(find.byType(TestText)).textStyle.color!;

    // Test updated ExpansionTile properties.
    expect(
      material.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(6))),
    );
    expect(material.color, const Color(0xff123456));
    expect(material.clipBehavior, Clip.antiAlias);
    expect(
      tester.state<TestIconState>(find.byType(TestIcon)).iconTheme.color,
      const Color(0xffffffff),
    );
    expect(
      tester.state<TestTextState>(find.byType(TestText)).textStyle.color,
      const Color(0xffffffff),
    );
  });

  testWidgets('Override ExpansionTile animation using AnimationStyle', (WidgetTester tester) async {
    const Key expansionTileKey = Key('expansionTileKey');

    Widget buildExpansionTile({AnimationStyle? animationStyle}) {
      return MaterialApp(
        home: Material(
          child: Center(
            child: ExpansionTile(
              key: expansionTileKey,
              expansionAnimationStyle: animationStyle,
              title: const TestText('title'),
              children: const <Widget>[SizedBox(height: 100, width: 100)],
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildExpansionTile());

    double getHeight(Key key) => tester.getSize(find.byKey(key)).height;

    // Test initial ExpansionTile height.
    expect(getHeight(expansionTileKey), 58.0);

    // Test the default expansion animation.
    await tester.tap(find.text('title'));
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 50),
    ); // Advance the animation by 1/4 of its duration.

    expect(getHeight(expansionTileKey), closeTo(67.4, 0.1));

    await tester.pump(
      const Duration(milliseconds: 50),
    ); // Advance the animation by 2/4 of its duration.

    expect(getHeight(expansionTileKey), closeTo(89.6, 0.1));

    await tester.pumpAndSettle(); // Advance the animation to the end.

    expect(getHeight(expansionTileKey), 158.0);

    // Tap to collapse the ExpansionTile.
    await tester.tap(find.text('title'));
    await tester.pumpAndSettle();

    // Override the animation duration.
    await tester.pumpWidget(
      buildExpansionTile(
        animationStyle: AnimationStyle(duration: const Duration(milliseconds: 800)),
      ),
    );
    await tester.pumpAndSettle();

    // Test the overridden animation duration.
    await tester.tap(find.text('title'));
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 200),
    ); // Advance the animation by 1/4 of its duration.

    expect(getHeight(expansionTileKey), closeTo(67.4, 0.1));

    await tester.pump(
      const Duration(milliseconds: 200),
    ); // Advance the animation by 2/4 of its duration.

    expect(getHeight(expansionTileKey), closeTo(89.6, 0.1));

    await tester.pumpAndSettle(); // Advance the animation to the end.

    expect(getHeight(expansionTileKey), 158.0);

    // Tap to collapse the ExpansionTile.
    await tester.tap(find.text('title'));
    await tester.pumpAndSettle();

    // Override the animation curve.
    await tester.pumpWidget(
      buildExpansionTile(
        animationStyle: AnimationStyle(
          curve: Easing.emphasizedDecelerate,
          reverseCurve: Easing.emphasizedAccelerate,
        ),
      ),
    );
    await tester.pumpAndSettle();

    // Test the overridden animation curve.
    await tester.tap(find.text('title'));
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 50),
    ); // Advance the animation by 1/4 of its duration.

    expect(getHeight(expansionTileKey), closeTo(141.2, 0.1));

    await tester.pump(
      const Duration(milliseconds: 50),
    ); // Advance the animation by 2/4 of its duration.

    expect(getHeight(expansionTileKey), closeTo(153, 0.1));

    await tester.pumpAndSettle(); // Advance the animation to the end.

    expect(getHeight(expansionTileKey), 158.0);

    // Test the overridden reverse (collapse) animation curve.
    await tester.tap(find.text('title'));
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 50),
    ); // Advance the animation by 1/4 of its duration.

    expect(getHeight(expansionTileKey), closeTo(98.6, 0.1));

    await tester.pump(
      const Duration(milliseconds: 50),
    ); // Advance the animation by 2/4 of its duration.

    expect(getHeight(expansionTileKey), closeTo(73.4, 0.1));

    await tester.pumpAndSettle(); // Advance the animation to the end.

    expect(getHeight(expansionTileKey), 58.0);

    // Test no animation.
    await tester.pumpWidget(buildExpansionTile(animationStyle: AnimationStyle.noAnimation));

    // Tap to expand the ExpansionTile.
    await tester.tap(find.text('title'));
    await tester.pump();

    expect(getHeight(expansionTileKey), 158.0);
  });

  testWidgets('Material3 - ExpansionTile draws Inkwell splash on top of background color', (
    WidgetTester tester,
  ) async {
    const Key expansionTileKey = Key('expansionTileKey');
    const ShapeBorder shape = RoundedRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(16)),
    );
    const ShapeBorder collapsedShape = RoundedRectangleBorder(
      borderRadius: BorderRadius.all(Radius.circular(16)),
    );
    const Color collapsedBackgroundColor = Color(0xff00ff00);
    const Color backgroundColor = Color(0xffff0000);

    await tester.pumpWidget(
      const MaterialApp(
        home: Material(
          child: Center(
            child: Padding(
              padding: EdgeInsets.symmetric(horizontal: 24.0),
              child: ExpansionTile(
                key: expansionTileKey,
                shape: shape,
                collapsedBackgroundColor: collapsedBackgroundColor,
                backgroundColor: backgroundColor,
                collapsedShape: collapsedShape,
                title: TestText('title'),
                trailing: TestIcon(),
                children: <Widget>[SizedBox(height: 100, width: 100)],
              ),
            ),
          ),
        ),
      ),
    );

    // Tap and hold the ExpansionTile to trigger ink splash.
    final Offset center = tester.getCenter(find.byKey(expansionTileKey));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pump(); // Start the splash animation.
    await tester.pump(const Duration(milliseconds: 100)); // Splash is underway.

    // Material 3 uses the InkSparkle which uses a shader, so we can't capture
    // the effect with paint methods. Use a golden test instead.
    // Check if the ink sparkle is drawn on top of the background color.
    await expectLater(
      find.byKey(expansionTileKey),
      matchesGoldenFile('expansion_tile.ink_splash.drawn_on_top_of_background_color.png'),
    );

    // Finish gesture to release resources.
    await gesture.up();
    await tester.pumpAndSettle();
  });

  testWidgets('Default clipBehavior when a shape is provided', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: ExpansionTile(
            title: Text('Title'),
            subtitle: Text('Subtitle'),
            shape: StadiumBorder(),
            children: <Widget>[ListTile(title: Text('0'))],
          ),
        ),
      ),
    );

    expect(getMaterial(tester).clipBehavior, Clip.antiAlias);
  });

  testWidgets('Can override clipBehavior when a shape is provided', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: ExpansionTile(
            title: Text('Title'),
            subtitle: Text('Subtitle'),
            shape: StadiumBorder(),
            clipBehavior: Clip.none,
            children: <Widget>[ListTile(title: Text('0'))],
          ),
        ),
      ),
    );

    expect(getMaterial(tester).clipBehavior, Clip.none);
  });

  group('Material 2', () {
    // These tests are only relevant for Material 2. Once Material 2
    // support is deprecated and the APIs are removed, these tests
    // can be deleted.

    testWidgets('ExpansionTile default iconColor, textColor', (WidgetTester tester) async {
      final ThemeData theme = ThemeData(useMaterial3: false);

      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: const Material(
            child: ExpansionTile(
              title: TestText('title'),
              trailing: TestIcon(),
              children: <Widget>[SizedBox(height: 100, width: 100)],
            ),
          ),
        ),
      );

      Color getIconColor() => tester.state<TestIconState>(find.byType(TestIcon)).iconTheme.color!;
      Color getTextColor() => tester.state<TestTextState>(find.byType(TestText)).textStyle.color!;

      expect(getIconColor(), theme.unselectedWidgetColor);
      expect(getTextColor(), theme.textTheme.titleMedium!.color);

      await tester.tap(find.text('title'));
      await tester.pumpAndSettle();

      expect(getIconColor(), theme.colorScheme.primary);
      expect(getTextColor(), theme.colorScheme.primary);
    });

    testWidgets('Material2 - ExpansionTile draws inkwell splash on top of background color', (
      WidgetTester tester,
    ) async {
      const Key expansionTileKey = Key('expansionTileKey');
      final ThemeData theme = ThemeData(useMaterial3: false);
      const ShapeBorder shape = RoundedRectangleBorder(
        borderRadius: BorderRadius.all(Radius.circular(16)),
      );
      const ShapeBorder collapsedShape = RoundedRectangleBorder(
        borderRadius: BorderRadius.all(Radius.circular(16)),
      );
      const Color collapsedBackgroundColor = Color(0xff00ff00);
      const Color backgroundColor = Color(0xffff0000);

      await tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: const Material(
            child: Center(
              child: Padding(
                padding: EdgeInsets.symmetric(horizontal: 24.0),
                child: ExpansionTile(
                  key: expansionTileKey,
                  shape: shape,
                  collapsedBackgroundColor: collapsedBackgroundColor,
                  backgroundColor: backgroundColor,
                  collapsedShape: collapsedShape,
                  title: TestText('title'),
                  trailing: TestIcon(),
                  children: <Widget>[SizedBox(height: 100, width: 100)],
                ),
              ),
            ),
          ),
        ),
      );

      // Tap and hold the ExpansionTile to trigger ink splash.
      final Offset center = tester.getCenter(find.byKey(expansionTileKey));
      final TestGesture gesture = await tester.startGesture(center);
      await tester.pump(); // Start the splash animation.
      await tester.pump(const Duration(milliseconds: 100)); // Splash is underway.

      final RenderObject inkFeatures = tester.allRenderObjects.firstWhere(
        (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
      );
      // Check if the ink splash is drawn on top of the background color.
      expect(
        inkFeatures,
        paints
          ..path(color: collapsedBackgroundColor)
          ..circle(color: theme.splashColor),
      );

      // Finish gesture to release resources.
      await gesture.up();
      await tester.pumpAndSettle();
    });
  });

  testWidgets('ExpansionTileController isExpanded, expand() and collapse()', (
    WidgetTester tester,
  ) async {
    final ExpansionTileController controller = ExpansionTileController();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: ExpansionTile(
            controller: controller,
            title: const Text('Title'),
            children: const <Widget>[Text('Child 0')],
          ),
        ),
      ),
    );

    expect(find.text('Child 0'), findsNothing);
    expect(controller.isExpanded, isFalse);
    controller.expand();
    expect(controller.isExpanded, isTrue);
    await tester.pumpAndSettle();
    expect(find.text('Child 0'), findsOneWidget);
    expect(controller.isExpanded, isTrue);
    controller.collapse();
    expect(controller.isExpanded, isFalse);
    await tester.pumpAndSettle();
    expect(find.text('Child 0'), findsNothing);
  });

  testWidgets(
    'Calling ExpansionTileController.expand/collapsed has no effect if it is already expanded/collapsed',
    (WidgetTester tester) async {
      final ExpansionTileController controller = ExpansionTileController();

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: ExpansionTile(
              controller: controller,
              title: const Text('Title'),
              initiallyExpanded: true,
              children: const <Widget>[Text('Child 0')],
            ),
          ),
        ),
      );

      expect(find.text('Child 0'), findsOneWidget);
      expect(controller.isExpanded, isTrue);
      controller.expand();
      expect(controller.isExpanded, isTrue);
      await tester.pump();
      expect(tester.hasRunningAnimations, isFalse);
      expect(find.text('Child 0'), findsOneWidget);
      controller.collapse();
      expect(controller.isExpanded, isFalse);
      await tester.pump();
      expect(tester.hasRunningAnimations, isTrue);
      await tester.pumpAndSettle();
      expect(controller.isExpanded, isFalse);
      expect(find.text('Child 0'), findsNothing);
      controller.collapse();
      expect(controller.isExpanded, isFalse);
      await tester.pump();
      expect(tester.hasRunningAnimations, isFalse);
    },
  );

  testWidgets('Call to ExpansionTileController.of()', (WidgetTester tester) async {
    final GlobalKey titleKey = GlobalKey();
    final GlobalKey childKey = GlobalKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: ExpansionTile(
            initiallyExpanded: true,
            title: Text('Title', key: titleKey),
            children: <Widget>[Text('Child 0', key: childKey)],
          ),
        ),
      ),
    );

    final ExpansionTileController controller1 = ExpansionTileController.of(
      childKey.currentContext!,
    );
    expect(controller1.isExpanded, isTrue);

    final ExpansionTileController controller2 = ExpansionTileController.of(
      titleKey.currentContext!,
    );
    expect(controller2.isExpanded, isTrue);

    expect(controller1, controller2);
  });

  testWidgets('Call to ExpansionTile.maybeOf()', (WidgetTester tester) async {
    final GlobalKey titleKey = GlobalKey();
    final GlobalKey nonDescendantKey = GlobalKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Column(
            children: <Widget>[
              ExpansionTile(
                title: Text('Title', key: titleKey),
                children: const <Widget>[Text('Child 0')],
              ),
              Text('Non descendant', key: nonDescendantKey),
            ],
          ),
        ),
      ),
    );

    final ExpansionTileController? controller1 = ExpansionTileController.maybeOf(
      titleKey.currentContext!,
    );
    expect(controller1, isNotNull);
    expect(controller1?.isExpanded, isFalse);

    final ExpansionTileController? controller2 = ExpansionTileController.maybeOf(
      nonDescendantKey.currentContext!,
    );
    expect(controller2, isNull);
  });

  testWidgets('Check if dense, enableFeedback, visualDensity parameter is working', (
    WidgetTester tester,
  ) async {
    final GlobalKey titleKey = GlobalKey();
    final GlobalKey nonDescendantKey = GlobalKey();

    const bool dense = true;
    const bool enableFeedback = false;
    const VisualDensity visualDensity = VisualDensity.compact;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Column(
            children: <Widget>[
              ExpansionTile(
                dense: dense,
                enableFeedback: enableFeedback,
                visualDensity: visualDensity,
                title: Text('Title', key: titleKey),
                children: const <Widget>[Text('Child 0')],
              ),
              Text('Non descendant', key: nonDescendantKey),
            ],
          ),
        ),
      ),
    );

    final Finder tileFinder = find.byType(ListTile);
    final ListTile tileWidget = tester.widget<ListTile>(tileFinder);
    expect(tileWidget.dense, dense);
    expect(tileWidget.enableFeedback, enableFeedback);
    expect(tileWidget.visualDensity, visualDensity);
  });

  testWidgets('ExpansionTileController should not toggle if disabled', (WidgetTester tester) async {
    final ExpansionTileController controller = ExpansionTileController();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: ExpansionTile(
            enabled: false,
            controller: controller,
            title: const Text('Title'),
            children: const <Widget>[Text('Child 0')],
          ),
        ),
      ),
    );

    expect(find.text('Child 0'), findsNothing);
    expect(controller.isExpanded, isFalse);
    await tester.tap(find.widgetWithText(ExpansionTile, 'Title'));
    await tester.pumpAndSettle();
    expect(find.text('Child 0'), findsNothing);
    expect(controller.isExpanded, isFalse);
    controller.expand();
    await tester.pumpAndSettle();
    expect(find.text('Child 0'), findsOneWidget);
    expect(controller.isExpanded, isTrue);
    await tester.tap(find.widgetWithText(ExpansionTile, 'Title'));
    await tester.pumpAndSettle();
    expect(find.text('Child 0'), findsOneWidget);
    expect(controller.isExpanded, isTrue);
  });

  testWidgets(
    'ExpansionTile does not include the default trailing icon when showTrailingIcon: false (#145268)',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: ExpansionTile(
              enabled: false,
              tilePadding: EdgeInsets.zero,
              title: ColoredBox(color: Colors.red, child: Text('Title')),
              showTrailingIcon: false,
            ),
          ),
        ),
      );

      final Size materialAppSize = tester.getSize(find.byType(MaterialApp));
      final Size titleSize = tester.getSize(find.byType(ColoredBox));

      expect(titleSize.width, materialAppSize.width);
    },
  );

  testWidgets(
    'ExpansionTile with smaller trailing widget allocates at least 32.0 units of space (preserves original behavior) (#145268)',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(
          home: Material(
            child: ExpansionTile(
              enabled: false,
              tilePadding: EdgeInsets.zero,
              title: ColoredBox(color: Colors.red, child: Text('Title')),
              trailing: SizedBox.shrink(),
            ),
          ),
        ),
      );

      final Size materialAppSize = tester.getSize(find.byType(MaterialApp));
      final Size titleSize = tester.getSize(find.byType(ColoredBox));

      expect(titleSize.width, materialAppSize.width - 32.0);
    },
  );

  testWidgets('ExpansionTile uses ListTileTheme controlAffinity', (WidgetTester tester) async {
    Widget buildView(ListTileControlAffinity controlAffinity) {
      return MaterialApp(
        home: ListTileTheme(
          data: ListTileThemeData(controlAffinity: controlAffinity),
          child: const Material(child: ExpansionTile(title: Text('ExpansionTile'))),
        ),
      );
    }

    await tester.pumpWidget(buildView(ListTileControlAffinity.leading));
    final Finder leading = find.text('ExpansionTile');
    final Offset offsetLeading = tester.getTopLeft(leading);
    expect(offsetLeading, const Offset(56.0, 17.0));

    await tester.pumpWidget(buildView(ListTileControlAffinity.trailing));
    final Finder trailing = find.text('ExpansionTile');
    final Offset offsetTrailing = tester.getTopLeft(trailing);
    expect(offsetTrailing, const Offset(16.0, 17.0));

    await tester.pumpWidget(buildView(ListTileControlAffinity.platform));
    final Finder platform = find.text('ExpansionTile');
    final Offset offsetPlatform = tester.getTopLeft(platform);
    expect(offsetPlatform, const Offset(16.0, 17.0));
  });
}
