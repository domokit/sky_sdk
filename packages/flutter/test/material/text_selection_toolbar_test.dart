// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import '../widgets/editable_text_utils.dart' show textOffsetToPosition;

const double _kToolbarContentDistance = 8.0;

// A custom text selection menu that just displays a single custom button.
class _CustomMaterialTextSelectionControls extends MaterialTextSelectionControls {
  @override
  Widget buildToolbar(
    BuildContext context,
    Rect globalEditableRegion,
    double textLineHeight,
    Offset selectionMidpoint,
    List<TextSelectionPoint> endpoints,
    TextSelectionDelegate delegate,
    ValueListenable<ClipboardStatus>? clipboardStatus,
    Offset? lastSecondaryTapDownPosition,
  ) {
    final TextSelectionPoint startTextSelectionPoint = endpoints[0];
    final TextSelectionPoint endTextSelectionPoint =
        endpoints.length > 1 ? endpoints[1] : endpoints[0];
    final Offset anchorAbove = Offset(
      globalEditableRegion.left + selectionMidpoint.dx,
      globalEditableRegion.top +
          startTextSelectionPoint.point.dy -
          textLineHeight -
          _kToolbarContentDistance,
    );
    final Offset anchorBelow = Offset(
      globalEditableRegion.left + selectionMidpoint.dx,
      globalEditableRegion.top +
          endTextSelectionPoint.point.dy +
          TextSelectionToolbar.kToolbarContentDistanceBelow,
    );

    return TextSelectionToolbar(
      anchorAbove: anchorAbove,
      anchorBelow: anchorBelow,
      children: <Widget>[
        TextSelectionToolbarTextButton(
          padding: TextSelectionToolbarTextButton.getPadding(0, 1),
          onPressed: () {},
          child: const Text('Custom button'),
        ),
      ],
    );
  }
}

class TestBox extends SizedBox {
  const TestBox({super.key}) : super(width: itemWidth, height: itemHeight);

  static const double itemHeight = 44.0;
  static const double itemWidth = 100.0;
}

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  // Find by a runtimeType String, including private types.
  Finder findPrivate(String type) {
    return find.descendant(
      of: find.byType(MaterialApp),
      matching: find.byWidgetPredicate((Widget w) => '${w.runtimeType}' == type),
    );
  }

  // Finding TextSelectionToolbar won't give you the position as the user sees
  // it because it's a full-sized Stack at the top level. This method finds the
  // visible part of the toolbar for use in measurements.
  Finder findToolbar() => findPrivate('_TextSelectionToolbarOverflowable');

  Finder findOverflowButton() => findPrivate('_TextSelectionToolbarOverflowButton');

  testWidgets('puts children in an overflow menu if they overflow', (WidgetTester tester) async {
    late StateSetter setState;
    final List<Widget> children = List<Widget>.generate(7, (int i) => const TestBox());

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: StatefulBuilder(
            builder: (BuildContext context, StateSetter setter) {
              setState = setter;
              return TextSelectionToolbar(
                anchorAbove: const Offset(50.0, 100.0),
                anchorBelow: const Offset(50.0, 200.0),
                children: children,
              );
            },
          ),
        ),
      ),
    );

    // All children fit on the screen, so they are all rendered.
    expect(find.byType(TestBox), findsNWidgets(children.length));
    expect(findOverflowButton(), findsNothing);

    // Adding one more child makes the children overflow.
    setState(() {
      children.add(const TestBox());
    });
    await tester.pumpAndSettle();
    expect(find.byType(TestBox), findsNWidgets(children.length - 1));
    expect(findOverflowButton(), findsOneWidget);

    // Tap the overflow button to show the overflow menu.
    await tester.tap(findOverflowButton());
    await tester.pumpAndSettle();
    expect(find.byType(TestBox), findsNWidgets(1));
    expect(findOverflowButton(), findsOneWidget);

    // Tap the overflow button again to hide the overflow menu.
    await tester.tap(findOverflowButton());
    await tester.pumpAndSettle();
    expect(find.byType(TestBox), findsNWidgets(children.length - 1));
    expect(findOverflowButton(), findsOneWidget);
  });

  testWidgets('positions itself at anchorAbove if it fits', (WidgetTester tester) async {
    late StateSetter setState;
    const double height = 44.0;
    const double anchorBelowY = 500.0;
    double anchorAboveY = 0.0;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: StatefulBuilder(
            builder: (BuildContext context, StateSetter setter) {
              setState = setter;
              return TextSelectionToolbar(
                anchorAbove: Offset(50.0, anchorAboveY),
                anchorBelow: const Offset(50.0, anchorBelowY),
                children: <Widget>[
                  Container(color: Colors.red, width: 50.0, height: height),
                  Container(color: Colors.green, width: 50.0, height: height),
                  Container(color: Colors.blue, width: 50.0, height: height),
                ],
              );
            },
          ),
        ),
      ),
    );

    // When the toolbar doesn't fit above aboveAnchor, it positions itself below
    // belowAnchor.
    double toolbarY = tester.getTopLeft(findToolbar()).dy;
    expect(toolbarY, equals(anchorBelowY + TextSelectionToolbar.kToolbarContentDistanceBelow));

    // Even when it barely doesn't fit.
    setState(() {
      anchorAboveY = 60.0;
    });
    await tester.pump();
    toolbarY = tester.getTopLeft(findToolbar()).dy;
    expect(toolbarY, equals(anchorBelowY + TextSelectionToolbar.kToolbarContentDistanceBelow));

    // When it does fit above aboveAnchor, it positions itself there.
    setState(() {
      anchorAboveY = 70.0;
    });
    await tester.pump();
    toolbarY = tester.getTopLeft(findToolbar()).dy;
    expect(toolbarY, equals(anchorAboveY - height - _kToolbarContentDistance));
  });

  testWidgets('can create and use a custom toolbar', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: SelectableText(
              'Select me custom menu',
              selectionControls: _CustomMaterialTextSelectionControls(),
            ),
          ),
        ),
      ),
    );

    // The selection menu is not initially shown.
    expect(find.text('Custom button'), findsNothing);

    // Long press on "custom" to select it.
    final Offset customPos = textOffsetToPosition(tester, 11);
    final TestGesture gesture = await tester.startGesture(customPos, pointer: 7);
    await tester.pump(const Duration(seconds: 2));
    await gesture.up();
    await tester.pump();

    // The custom selection menu is shown.
    expect(find.text('Custom button'), findsOneWidget);
    expect(find.text('Cut'), findsNothing);
    expect(find.text('Copy'), findsNothing);
    expect(find.text('Paste'), findsNothing);
    expect(find.text('Select all'), findsNothing);
  }, skip: kIsWeb); // [intended] We don't show the toolbar on the web.

  for (final ColorScheme colorScheme in <ColorScheme>[
    ThemeData.light().colorScheme,
    ThemeData.dark().colorScheme,
  ]) {
    testWidgets('default background color', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(colorScheme: colorScheme),
          home: Scaffold(
            body: Center(
              child: TextSelectionToolbar(
                anchorAbove: Offset.zero,
                anchorBelow: Offset.zero,
                children: <Widget>[
                  TextSelectionToolbarTextButton(
                    padding: TextSelectionToolbarTextButton.getPadding(0, 1),
                    onPressed: () {},
                    child: const Text('Custom button'),
                  ),
                ],
              ),
            ),
          ),
        ),
      );

      Finder findToolbarContainer() {
        return find.descendant(
          of: find.byWidgetPredicate(
            (Widget w) => '${w.runtimeType}' == '_TextSelectionToolbarContainer',
          ),
          matching: find.byType(Material),
        );
      }

      expect(findToolbarContainer(), findsAtLeastNWidgets(1));

      final Material toolbarContainer = tester.widget(findToolbarContainer().first);
      expect(
        toolbarContainer.color,
        // The default colors are hardcoded and don't take the default value of
        // the theme's surface color.
        switch (colorScheme.brightness) {
          Brightness.light => const Color(0xffffffff),
          Brightness.dark => const Color(0xff424242),
        },
      );
    });

    testWidgets('custom background color', (WidgetTester tester) async {
      const Color customBackgroundColor = Colors.red;

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(colorScheme: colorScheme.copyWith(surface: customBackgroundColor)),
          home: Scaffold(
            body: Center(
              child: TextSelectionToolbar(
                anchorAbove: Offset.zero,
                anchorBelow: Offset.zero,
                children: <Widget>[
                  TextSelectionToolbarTextButton(
                    padding: TextSelectionToolbarTextButton.getPadding(0, 1),
                    onPressed: () {},
                    child: const Text('Custom button'),
                  ),
                ],
              ),
            ),
          ),
        ),
      );

      Finder findToolbarContainer() {
        return find.descendant(
          of: find.byWidgetPredicate(
            (Widget w) => '${w.runtimeType}' == '_TextSelectionToolbarContainer',
          ),
          matching: find.byType(Material),
        );
      }

      expect(findToolbarContainer(), findsAtLeastNWidgets(1));

      final Material toolbarContainer = tester.widget(findToolbarContainer().first);
      expect(toolbarContainer.color, customBackgroundColor);
    });
  }

  testWidgets('Overflowed menu expands children horizontally', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/144089.
    late StateSetter setState;
    final List<Widget> children = List<Widget>.generate(7, (int i) => const TestBox());

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: StatefulBuilder(
            builder: (BuildContext context, StateSetter setter) {
              setState = setter;
              return TextSelectionToolbar(
                anchorAbove: const Offset(50.0, 100.0),
                anchorBelow: const Offset(50.0, 200.0),
                children: children,
              );
            },
          ),
        ),
      ),
    );

    // All children fit on the screen, so they are all rendered.
    expect(find.byType(TestBox), findsNWidgets(children.length));
    expect(findOverflowButton(), findsNothing);

    const String short = 'Short';
    const String medium = 'Medium length';
    const String long = 'Long label in the overflow menu';

    // Adding several children makes the menu overflow.
    setState(() {
      children.addAll(const <Text>[Text(short), Text(medium), Text(long)]);
    });
    await tester.pumpAndSettle();
    expect(findOverflowButton(), findsOneWidget);

    // Tap the overflow button to show the overflow menu.
    await tester.tap(findOverflowButton());
    await tester.pumpAndSettle();
    expect(find.byType(TestBox), findsNothing);
    expect(find.byType(Text), findsNWidgets(3));
    expect(findOverflowButton(), findsOneWidget);

    Finder findToolbarContainer() {
      return find.byWidgetPredicate(
        (Widget w) => '${w.runtimeType}' == '_TextSelectionToolbarContainer',
      );
    }

    expect(findToolbarContainer(), findsAtLeastNWidgets(1));

    // Buttons have their width set to the container width.
    final double overflowMenuWidth = tester.getRect(findToolbarContainer()).width;
    expect(tester.getRect(find.text(long)).width, overflowMenuWidth);
    expect(tester.getRect(find.text(medium)).width, overflowMenuWidth);
    expect(tester.getRect(find.text(short)).width, overflowMenuWidth);
  });
}
