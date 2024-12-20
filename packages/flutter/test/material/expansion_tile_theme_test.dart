// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

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
  Material getMaterial(WidgetTester tester) {
    return tester.widget<Material>(
      find.descendant(of: find.byType(ExpansionTile), matching: find.byType(Material)),
    );
  }

  test('ExpansionTileThemeData copyWith, ==, hashCode basics', () {
    expect(const ExpansionTileThemeData(), const ExpansionTileThemeData().copyWith());
    expect(
      const ExpansionTileThemeData().hashCode,
      const ExpansionTileThemeData().copyWith().hashCode,
    );
  });

  test('ExpansionTileThemeData lerp special cases', () {
    expect(ExpansionTileThemeData.lerp(null, null, 0), null);
    const ExpansionTileThemeData data = ExpansionTileThemeData();
    expect(identical(ExpansionTileThemeData.lerp(data, data, 0.5), data), true);
  });

  test('ExpansionTileThemeData defaults', () {
    const ExpansionTileThemeData theme = ExpansionTileThemeData();
    expect(theme.backgroundColor, null);
    expect(theme.collapsedBackgroundColor, null);
    expect(theme.tilePadding, null);
    expect(theme.expandedAlignment, null);
    expect(theme.childrenPadding, null);
    expect(theme.iconColor, null);
    expect(theme.collapsedIconColor, null);
    expect(theme.textColor, null);
    expect(theme.collapsedTextColor, null);
    expect(theme.shape, null);
    expect(theme.collapsedShape, null);
    expect(theme.clipBehavior, null);
    expect(theme.expansionAnimationStyle, null);
  });

  testWidgets('Default ExpansionTileThemeData debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const TooltipThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('ExpansionTileThemeData implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    ExpansionTileThemeData(
      backgroundColor: const Color(0xff000000),
      collapsedBackgroundColor: const Color(0xff6f83fc),
      tilePadding: const EdgeInsets.all(20.0),
      expandedAlignment: Alignment.bottomCenter,
      childrenPadding: const EdgeInsets.all(10.0),
      iconColor: const Color(0xffa7c61c),
      collapsedIconColor: const Color(0xffdd0b1f),
      textColor: const Color(0xffffffff),
      collapsedTextColor: const Color(0xff522bab),
      shape: const Border(),
      collapsedShape: const Border(),
      clipBehavior: Clip.antiAlias,
      expansionAnimationStyle: AnimationStyle(curve: Curves.easeInOut),
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(
      description,
      equalsIgnoringHashCodes(<String>[
        'backgroundColor: ${const Color(0xff000000)}',
        'collapsedBackgroundColor: ${const Color(0xff6f83fc)}',
        'tilePadding: EdgeInsets.all(20.0)',
        'expandedAlignment: Alignment.bottomCenter',
        'childrenPadding: EdgeInsets.all(10.0)',
        'iconColor: ${const Color(0xffa7c61c)}',
        'collapsedIconColor: ${const Color(0xffdd0b1f)}',
        'textColor: ${const Color(0xffffffff)}',
        'collapsedTextColor: ${const Color(0xff522bab)}',
        'shape: Border.all(BorderSide(width: 0.0, style: none))',
        'collapsedShape: Border.all(BorderSide(width: 0.0, style: none))',
        'clipBehavior: Clip.antiAlias',
        'expansionAnimationStyle: AnimationStyle#983ac(curve: Cubic(0.42, 0.00, 0.58, 1.00))',
      ]),
    );
  });

  testWidgets('ExpansionTileTheme - collapsed', (WidgetTester tester) async {
    final Key tileKey = UniqueKey();
    final Key titleKey = UniqueKey();
    final Key iconKey = UniqueKey();
    const Color backgroundColor = Colors.orange;
    const Color collapsedBackgroundColor = Colors.red;
    const Color iconColor = Colors.green;
    const Color collapsedIconColor = Colors.blue;
    const Color textColor = Colors.black;
    const Color collapsedTextColor = Colors.white;
    const ShapeBorder shape = Border(
      top: BorderSide(color: Colors.red),
      bottom: BorderSide(color: Colors.red),
    );
    const ShapeBorder collapsedShape = Border(
      top: BorderSide(color: Colors.green),
      bottom: BorderSide(color: Colors.green),
    );
    const Clip clipBehavior = Clip.antiAlias;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          expansionTileTheme: const ExpansionTileThemeData(
            backgroundColor: backgroundColor,
            collapsedBackgroundColor: collapsedBackgroundColor,
            tilePadding: EdgeInsets.fromLTRB(8, 12, 4, 10),
            expandedAlignment: Alignment.centerRight,
            childrenPadding: EdgeInsets.all(20.0),
            iconColor: iconColor,
            collapsedIconColor: collapsedIconColor,
            textColor: textColor,
            collapsedTextColor: collapsedTextColor,
            shape: shape,
            collapsedShape: collapsedShape,
            clipBehavior: clipBehavior,
          ),
        ),
        home: Material(
          child: Center(
            child: ExpansionTile(
              key: tileKey,
              title: TestText('Collapsed Tile', key: titleKey),
              trailing: TestIcon(key: iconKey),
              children: const <Widget>[Text('Tile 1')],
            ),
          ),
        ),
      ),
    );

    // When a custom shape is provided, ExpansionTile will use the
    // Material widget to draw the shape and background color
    // instead of a Container.
    final Material material = getMaterial(tester);

    // ExpansionTile should have Clip.antiAlias as clipBehavior.
    expect(material.clipBehavior, clipBehavior);

    // Check the tile's collapsed background color when collapsedBackgroundColor is applied.
    expect(material.color, collapsedBackgroundColor);

    final Rect titleRect = tester.getRect(find.text('Collapsed Tile'));
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

    Color getIconColor() => tester.state<TestIconState>(find.byType(TestIcon)).iconTheme.color!;
    Color getTextColor() => tester.state<TestTextState>(find.byType(TestText)).textStyle.color!;

    // Check the collapsed icon color when iconColor is applied.
    expect(getIconColor(), collapsedIconColor);
    // Check the collapsed text color when textColor is applied.
    expect(getTextColor(), collapsedTextColor);
    // Check the collapsed ShapeBorder when shape is applied.
    expect(material.shape, collapsedShape);
  });

  testWidgets('ExpansionTileTheme - expanded', (WidgetTester tester) async {
    final Key tileKey = UniqueKey();
    final Key titleKey = UniqueKey();
    final Key iconKey = UniqueKey();
    const Color backgroundColor = Colors.orange;
    const Color collapsedBackgroundColor = Colors.red;
    const Color iconColor = Colors.green;
    const Color collapsedIconColor = Colors.blue;
    const Color textColor = Colors.black;
    const Color collapsedTextColor = Colors.white;
    const ShapeBorder shape = Border(
      top: BorderSide(color: Colors.red),
      bottom: BorderSide(color: Colors.red),
    );
    const ShapeBorder collapsedShape = Border(
      top: BorderSide(color: Colors.green),
      bottom: BorderSide(color: Colors.green),
    );
    const Clip clipBehavior = Clip.none;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          expansionTileTheme: const ExpansionTileThemeData(
            backgroundColor: backgroundColor,
            collapsedBackgroundColor: collapsedBackgroundColor,
            tilePadding: EdgeInsets.fromLTRB(8, 12, 4, 10),
            expandedAlignment: Alignment.centerRight,
            childrenPadding: EdgeInsets.all(20.0),
            iconColor: iconColor,
            collapsedIconColor: collapsedIconColor,
            textColor: textColor,
            collapsedTextColor: collapsedTextColor,
            shape: shape,
            collapsedShape: collapsedShape,
            clipBehavior: clipBehavior,
          ),
        ),
        home: Material(
          child: Center(
            child: ExpansionTile(
              key: tileKey,
              initiallyExpanded: true,
              title: TestText('Expanded Tile', key: titleKey),
              trailing: TestIcon(key: iconKey),
              children: const <Widget>[Text('Tile 1')],
            ),
          ),
        ),
      ),
    );

    // When a custom shape is provided, ExpansionTile will use the
    // Material widget to draw the shape and background color
    // instead of a Container.
    final Material material = getMaterial(tester);
    // Check the tile's background color when backgroundColor is applied.
    expect(material.color, backgroundColor);

    final Rect titleRect = tester.getRect(find.text('Expanded Tile'));
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

    Color getIconColor() => tester.state<TestIconState>(find.byType(TestIcon)).iconTheme.color!;
    Color getTextColor() => tester.state<TestTextState>(find.byType(TestText)).textStyle.color!;

    // Check the expanded icon color when iconColor is applied.
    expect(getIconColor(), iconColor);
    // Check the expanded text color when textColor is applied.
    expect(getTextColor(), textColor);
    // Check the expanded ShapeBorder when shape is applied.
    expect(material.shape, shape);
    // Check the clipBehavior when shape is applied.
    expect(material.clipBehavior, clipBehavior);

    // Check the child position when expandedAlignment is applied.
    final Rect childRect = tester.getRect(find.text('Tile 1'));
    expect(childRect.right, 800 - 20);
    expect(childRect.left, 800 - childRect.width - 20);

    // Check the child padding when childrenPadding is applied.
    final Rect paddingRect = tester.getRect(find.byType(Padding).last);
    expect(childRect.top, paddingRect.top + 20);
    expect(childRect.left, paddingRect.left + 20);
    expect(childRect.right, paddingRect.right - 20);
    expect(childRect.bottom, paddingRect.bottom - 20);
  });

  testWidgets('Override ExpansionTile animation using ExpansionTileThemeData.AnimationStyle', (
    WidgetTester tester,
  ) async {
    const Key expansionTileKey = Key('expansionTileKey');

    Widget buildExpansionTile({AnimationStyle? animationStyle}) {
      return MaterialApp(
        theme: ThemeData(
          expansionTileTheme: ExpansionTileThemeData(expansionAnimationStyle: animationStyle),
        ),
        home: const Material(
          child: Center(
            child: ExpansionTile(
              key: expansionTileKey,
              title: TestText('title'),
              children: <Widget>[SizedBox(height: 100, width: 100)],
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
      buildExpansionTile(animationStyle: AnimationStyle(curve: Easing.emphasizedDecelerate)),
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

    // Tap to collapse the ExpansionTile.
    await tester.tap(find.text('title'));

    // Test no animation.
    await tester.pumpWidget(buildExpansionTile(animationStyle: AnimationStyle.noAnimation));

    // Tap to expand the ExpansionTile.
    await tester.tap(find.text('title'));
    await tester.pump();

    expect(getHeight(expansionTileKey), 158.0);
  });
}
