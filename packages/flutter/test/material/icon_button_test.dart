// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/feedback_tester.dart';
import '../widgets/semantics_tester.dart';

class MockOnPressedFunction {
  int called = 0;

  void handler() {
    called++;
  }
}

void main() {
  late MockOnPressedFunction mockOnPressedFunction;
  const ColorScheme colorScheme = ColorScheme.light();
  final ThemeData theme = ThemeData.from(colorScheme: colorScheme);
  setUp(() {
    mockOnPressedFunction = MockOnPressedFunction();
  });

  RenderObject getOverlayColor(WidgetTester tester) {
    return tester.allRenderObjects.firstWhere(
      (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
    );
  }

  Finder findTooltipContainer(String tooltipText) {
    return find.ancestor(of: find.text(tooltipText), matching: find.byType(Container));
  }

  testWidgets('test icon is findable by key', (WidgetTester tester) async {
    const ValueKey<String> key = ValueKey<String>('icon-button');
    await tester.pumpWidget(
      wrap(
        useMaterial3: true,
        child: IconButton(key: key, onPressed: () {}, icon: const Icon(Icons.link)),
      ),
    );

    expect(find.byKey(key), findsOneWidget);
  });

  testWidgets('test default icon buttons are sized up to 48', (WidgetTester tester) async {
    final bool material3 = theme.useMaterial3;
    await tester.pumpWidget(
      wrap(
        useMaterial3: material3,
        child: IconButton(onPressed: mockOnPressedFunction.handler, icon: const Icon(Icons.link)),
      ),
    );

    final RenderBox iconButton = tester.renderObject(find.byType(IconButton));
    expect(iconButton.size, const Size(48.0, 48.0));

    await tester.tap(find.byType(IconButton));
    expect(mockOnPressedFunction.called, 1);
  });

  testWidgets('test small icons are sized up to 48dp', (WidgetTester tester) async {
    final bool material3 = theme.useMaterial3;
    await tester.pumpWidget(
      wrap(
        useMaterial3: material3,
        child: IconButton(
          iconSize: 10.0,
          onPressed: mockOnPressedFunction.handler,
          icon: const Icon(Icons.link),
        ),
      ),
    );

    final RenderBox iconButton = tester.renderObject(find.byType(IconButton));
    expect(iconButton.size, const Size(48.0, 48.0));
  });

  testWidgets('test icons can be small when total size is >48dp', (WidgetTester tester) async {
    final bool material3 = theme.useMaterial3;
    await tester.pumpWidget(
      wrap(
        useMaterial3: material3,
        child: IconButton(
          iconSize: 10.0,
          padding: const EdgeInsets.all(30.0),
          onPressed: mockOnPressedFunction.handler,
          icon: const Icon(Icons.link),
        ),
      ),
    );

    final RenderBox iconButton = tester.renderObject(find.byType(IconButton));
    expect(iconButton.size, const Size(70.0, 70.0));
  });

  testWidgets(
    'when both iconSize and IconTheme.of(context).size are null, size falls back to 24.0',
    (WidgetTester tester) async {
      final bool material3 = theme.useMaterial3;
      final FocusNode focusNode = FocusNode(debugLabel: 'Ink Focus');
      await tester.pumpWidget(
        wrap(
          useMaterial3: material3,
          child: IconTheme(
            data: const IconThemeData(),
            child: IconButton(
              focusNode: focusNode,
              onPressed: mockOnPressedFunction.handler,
              icon: const Icon(Icons.link),
            ),
          ),
        ),
      );

      final RenderBox icon = tester.renderObject(find.byType(Icon));
      expect(icon.size, const Size(24.0, 24.0));

      focusNode.dispose();
    },
  );

  testWidgets('when null, iconSize is overridden by closest IconTheme', (
    WidgetTester tester,
  ) async {
    RenderBox icon;
    final bool material3 = theme.useMaterial3;

    await tester.pumpWidget(
      wrap(
        useMaterial3: material3,
        child: IconTheme(
          data: const IconThemeData(size: 10),
          child: IconButton(onPressed: mockOnPressedFunction.handler, icon: const Icon(Icons.link)),
        ),
      ),
    );

    icon = tester.renderObject(find.byType(Icon));
    expect(icon.size, const Size(10.0, 10.0));

    await tester.pumpWidget(
      wrap(
        useMaterial3: material3,
        child: Theme(
          data: ThemeData(useMaterial3: material3, iconTheme: const IconThemeData(size: 10)),
          child: IconButton(onPressed: mockOnPressedFunction.handler, icon: const Icon(Icons.link)),
        ),
      ),
    );

    icon = tester.renderObject(find.byType(Icon));
    expect(icon.size, const Size(10.0, 10.0));

    await tester.pumpWidget(
      wrap(
        useMaterial3: material3,
        child: Theme(
          data: ThemeData(useMaterial3: material3, iconTheme: const IconThemeData(size: 20)),
          child: IconTheme(
            data: const IconThemeData(size: 10),
            child: IconButton(
              onPressed: mockOnPressedFunction.handler,
              icon: const Icon(Icons.link),
            ),
          ),
        ),
      ),
    );

    icon = tester.renderObject(find.byType(Icon));
    expect(icon.size, const Size(10.0, 10.0));

    await tester.pumpWidget(
      wrap(
        useMaterial3: material3,
        child: IconTheme(
          data: const IconThemeData(size: 20),
          child: Theme(
            data: ThemeData(useMaterial3: material3, iconTheme: const IconThemeData(size: 10)),
            child: IconButton(
              onPressed: mockOnPressedFunction.handler,
              icon: const Icon(Icons.link),
            ),
          ),
        ),
      ),
    );

    icon = tester.renderObject(find.byType(Icon));
    expect(icon.size, const Size(10.0, 10.0));
  });

  testWidgets('when non-null, iconSize precedes IconTheme.of(context).size', (
    WidgetTester tester,
  ) async {
    final bool material3 = theme.useMaterial3;
    await tester.pumpWidget(
      wrap(
        useMaterial3: material3,
        child: IconTheme(
          data: const IconThemeData(size: 30.0),
          child: IconButton(
            iconSize: 10.0,
            onPressed: mockOnPressedFunction.handler,
            icon: const Icon(Icons.link),
          ),
        ),
      ),
    );

    final RenderBox icon = tester.renderObject(find.byType(Icon));
    expect(icon.size, const Size(10.0, 10.0));
  });

  testWidgets('Small icons with non-null constraints can be <48dp for M2, but =48dp for M3', (
    WidgetTester tester,
  ) async {
    final bool material3 = theme.useMaterial3;
    await tester.pumpWidget(
      wrap(
        useMaterial3: material3,
        child: IconButton(
          iconSize: 10.0,
          onPressed: mockOnPressedFunction.handler,
          icon: const Icon(Icons.link),
          constraints: const BoxConstraints(),
        ),
      ),
    );

    final RenderBox iconButton = tester.renderObject(find.byType(IconButton));
    final RenderBox icon = tester.renderObject(find.byType(Icon));

    // By default IconButton has a padding of 8.0 on all sides, so both
    // width and height are 10.0 + 2 * 8.0 = 26.0
    // M3 IconButton is a subclass of ButtonStyleButton which has a minimum
    // Size(48.0, 48.0).
    expect(iconButton.size, material3 ? const Size(48.0, 48.0) : const Size(26.0, 26.0));
    expect(icon.size, const Size(10.0, 10.0));
  });

  testWidgets('Small icons with non-null constraints and custom padding can be <48dp', (
    WidgetTester tester,
  ) async {
    final bool material3 = theme.useMaterial3;
    await tester.pumpWidget(
      wrap(
        useMaterial3: material3,
        child: IconButton(
          iconSize: 10.0,
          padding: const EdgeInsets.all(3.0),
          onPressed: mockOnPressedFunction.handler,
          icon: const Icon(Icons.link),
          constraints: const BoxConstraints(),
        ),
      ),
    );

    final RenderBox iconButton = tester.renderObject(find.byType(IconButton));
    final RenderBox icon = tester.renderObject(find.byType(Icon));

    // This IconButton has a padding of 3.0 on all sides, so both
    // width and height are 10.0 + 2 * 3.0 = 16.0
    // M3 IconButton is a subclass of ButtonStyleButton which has a minimum
    // Size(48.0, 48.0).
    expect(iconButton.size, material3 ? const Size(48.0, 48.0) : const Size(16.0, 16.0));
    expect(icon.size, const Size(10.0, 10.0));
  });

  testWidgets('Small icons comply with VisualDensity requirements', (WidgetTester tester) async {
    final bool material3 = theme.useMaterial3;
    final ThemeData themeDataM2 = ThemeData(
      useMaterial3: material3,
      visualDensity: const VisualDensity(horizontal: 1, vertical: -1),
    );
    final ThemeData themeDataM3 = ThemeData(
      useMaterial3: material3,
      iconButtonTheme: IconButtonThemeData(
        style: IconButton.styleFrom(
          visualDensity: const VisualDensity(horizontal: 1, vertical: -1),
        ),
      ),
    );
    await tester.pumpWidget(
      wrap(
        useMaterial3: material3,
        child: Theme(
          data: material3 ? themeDataM3 : themeDataM2,
          child: IconButton(
            iconSize: 10.0,
            onPressed: mockOnPressedFunction.handler,
            icon: const Icon(Icons.link),
            constraints: const BoxConstraints(minWidth: 32.0, minHeight: 32.0),
          ),
        ),
      ),
    );

    final RenderBox iconButton = tester.renderObject(find.byType(IconButton));

    // VisualDensity(horizontal: 1, vertical: -1) increases the icon's
    // width by 4 pixels and decreases its height by 4 pixels, giving
    // final width 32.0 + 4.0 = 36.0 and
    // final height 32.0 - 4.0 = 28.0
    expect(iconButton.size, material3 ? const Size(52.0, 44.0) : const Size(36.0, 28.0));
  });

  testWidgets('test default icon buttons are constrained', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        useMaterial3: theme.useMaterial3,
        child: IconButton(
          padding: EdgeInsets.zero,
          onPressed: mockOnPressedFunction.handler,
          icon: const Icon(Icons.ac_unit),
          iconSize: 80.0,
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byType(IconButton));
    expect(box.size, const Size(80.0, 80.0));
  });

  testWidgets('test default icon buttons can be stretched if specified', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Material(
          child: Row(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: <Widget>[
              IconButton(onPressed: mockOnPressedFunction.handler, icon: const Icon(Icons.ac_unit)),
            ],
          ),
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byType(IconButton));
    expect(box.size, const Size(48.0, 600.0));

    // Test for Material 3
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.from(colorScheme: colorScheme),
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Row(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: <Widget>[
              IconButton(onPressed: mockOnPressedFunction.handler, icon: const Icon(Icons.ac_unit)),
            ],
          ),
        ),
      ),
    );

    final RenderBox boxM3 = tester.renderObject(find.byType(IconButton));
    expect(boxM3.size, const Size(48.0, 600.0));
  });

  testWidgets('test default padding', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        useMaterial3: theme.useMaterial3,
        child: IconButton(
          onPressed: mockOnPressedFunction.handler,
          icon: const Icon(Icons.ac_unit),
          iconSize: 80.0,
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byType(IconButton));
    expect(box.size, const Size(96.0, 96.0));
  });

  testWidgets('test default alignment', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        useMaterial3: theme.useMaterial3,
        child: IconButton(
          onPressed: mockOnPressedFunction.handler,
          icon: const Icon(Icons.ac_unit),
          iconSize: 80.0,
        ),
      ),
    );

    final Align align = tester.firstWidget<Align>(
      find.ancestor(of: find.byIcon(Icons.ac_unit), matching: find.byType(Align)),
    );
    expect(align.alignment, Alignment.center);
  });

  testWidgets('test tooltip', (WidgetTester tester) async {
    const String tooltipText = 'Test tooltip';
    Widget buildIconButton({String? tooltip}) {
      return MaterialApp(
        theme: theme,
        home: Material(
          child: Center(
            child: IconButton(
              onPressed: mockOnPressedFunction.handler,
              icon: const Icon(Icons.ac_unit),
              tooltip: tooltip,
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildIconButton());

    expect(find.byType(Tooltip), findsNothing);

    // Clear the widget tree.
    await tester.pumpWidget(Container(key: UniqueKey()));

    await tester.pumpWidget(buildIconButton(tooltip: tooltipText));

    expect(find.byType(Tooltip), findsOneWidget);
    expect(find.byTooltip(tooltipText), findsOneWidget);

    await tester.tap(find.byTooltip(tooltipText));
    expect(mockOnPressedFunction.called, 1);

    // Hovering over the button should show the tooltip.
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: tester.getCenter(find.byType(IconButton)));
    await tester.pump();

    expect(findTooltipContainer(tooltipText), findsOneWidget);
  });

  testWidgets('IconButton AppBar size', (WidgetTester tester) async {
    final bool material3 = theme.useMaterial3;
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          appBar: AppBar(
            actions: <Widget>[
              IconButton(
                padding: EdgeInsets.zero,
                onPressed: mockOnPressedFunction.handler,
                icon: const Icon(Icons.ac_unit),
              ),
            ],
          ),
        ),
      ),
    );

    final RenderBox barBox = tester.renderObject(find.byType(AppBar));
    final RenderBox iconBox = tester.renderObject(find.byType(IconButton));
    expect(iconBox.size.height, material3 ? 48 : equals(barBox.size.height));
    expect(tester.getCenter(find.byType(IconButton)).dy, 28);
  });

  // This test is very similar to the '...explicit splashColor and highlightColor' test
  // in buttons_test.dart. If you change this one, you may want to also change that one.
  testWidgets('IconButton with explicit splashColor and highlightColor - M2', (
    WidgetTester tester,
  ) async {
    const Color directSplashColor = Color(0xFF00000F);
    const Color directHighlightColor = Color(0xFF0000F0);

    Widget buttonWidget = wrap(
      useMaterial3: false,
      child: IconButton(
        icon: const Icon(Icons.android),
        splashColor: directSplashColor,
        highlightColor: directHighlightColor,
        onPressed: () {
          /* enable the button */
        },
      ),
    );

    await tester.pumpWidget(Theme(data: ThemeData(useMaterial3: false), child: buttonWidget));

    final Offset center = tester.getCenter(find.byType(IconButton));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pump(); // start gesture
    await tester.pump(const Duration(milliseconds: 200)); // wait for splash to be well under way

    expect(
      Material.of(tester.element(find.byType(IconButton))),
      paints
        ..circle(color: directSplashColor)
        ..circle(color: directHighlightColor),
    );

    const Color themeSplashColor1 = Color(0xFF000F00);
    const Color themeHighlightColor1 = Color(0xFF00FF00);

    buttonWidget = wrap(
      useMaterial3: false,
      child: IconButton(
        icon: const Icon(Icons.android),
        onPressed: () {
          /* enable the button */
        },
      ),
    );

    await tester.pumpWidget(
      Theme(
        data: ThemeData(
          highlightColor: themeHighlightColor1,
          splashColor: themeSplashColor1,
          useMaterial3: false,
        ),
        child: buttonWidget,
      ),
    );

    expect(
      Material.of(tester.element(find.byType(IconButton))),
      paints
        ..circle(color: themeSplashColor1)
        ..circle(color: themeHighlightColor1),
    );

    const Color themeSplashColor2 = Color(0xFF002200);
    const Color themeHighlightColor2 = Color(0xFF001100);

    await tester.pumpWidget(
      Theme(
        data: ThemeData(
          highlightColor: themeHighlightColor2,
          splashColor: themeSplashColor2,
          useMaterial3: false,
        ),
        child: buttonWidget, // same widget, so does not get updated because of us
      ),
    );

    expect(
      Material.of(tester.element(find.byType(IconButton))),
      paints
        ..circle(color: themeSplashColor2)
        ..circle(color: themeHighlightColor2),
    );

    await gesture.up();
  });

  testWidgets('IconButton with explicit splash radius - M2', (WidgetTester tester) async {
    const double splashRadius = 30.0;
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Material(
          child: Center(
            child: IconButton(
              icon: const Icon(Icons.android),
              splashRadius: splashRadius,
              onPressed: () {
                /* enable the button */
              },
            ),
          ),
        ),
      ),
    );

    final Offset center = tester.getCenter(find.byType(IconButton));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pump(); // Start gesture.
    await tester.pump(const Duration(milliseconds: 1000)); // Wait for splash to be well under way.

    expect(
      Material.of(tester.element(find.byType(IconButton))),
      paints..circle(radius: splashRadius),
    );

    await gesture.up();
  });

  testWidgets('IconButton Semantics (enabled) - M2', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      wrap(
        useMaterial3: false,
        child: IconButton(
          onPressed: mockOnPressedFunction.handler,
          icon: const Icon(Icons.link, semanticLabel: 'link'),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              rect: const Rect.fromLTRB(0.0, 0.0, 48.0, 48.0),
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
              flags: <SemanticsFlag>[
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isButton,
                SemanticsFlag.isEnabled,
                SemanticsFlag.isFocusable,
              ],
              label: 'link',
            ),
          ],
        ),
        ignoreId: true,
        ignoreTransform: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('IconButton Semantics (disabled) - M2', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      wrap(
        useMaterial3: false,
        child: const IconButton(onPressed: null, icon: Icon(Icons.link, semanticLabel: 'link')),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics.rootChild(
              rect: const Rect.fromLTRB(0.0, 0.0, 48.0, 48.0),
              flags: <SemanticsFlag>[SemanticsFlag.hasEnabledState, SemanticsFlag.isButton],
              label: 'link',
            ),
          ],
        ),
        ignoreId: true,
        ignoreTransform: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('IconButton Semantics (selected) - M3', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      wrap(
        useMaterial3: true,
        child: IconButton(
          onPressed: mockOnPressedFunction.handler,
          isSelected: true,
          icon: const Icon(Icons.link, semanticLabel: 'link'),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
                          flags: <SemanticsFlag>[
                            SemanticsFlag.hasEnabledState,
                            SemanticsFlag.isButton,
                            SemanticsFlag.isEnabled,
                            SemanticsFlag.isFocusable,
                            SemanticsFlag.hasSelectedState,
                            SemanticsFlag.isSelected,
                          ],
                          label: 'link',
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
          ],
        ),
        ignoreId: true,
        ignoreRect: true,
        ignoreTransform: true,
      ),
    );
    semantics.dispose();
  });

  testWidgets('IconButton loses focus when disabled.', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'IconButton');
    await tester.pumpWidget(
      wrap(
        useMaterial3: theme.useMaterial3,
        child: IconButton(
          focusNode: focusNode,
          autofocus: true,
          onPressed: () {},
          icon: const Icon(Icons.link),
        ),
      ),
    );

    await tester.pump();
    expect(focusNode.hasPrimaryFocus, isTrue);

    await tester.pumpWidget(
      wrap(
        useMaterial3: theme.useMaterial3,
        child: IconButton(
          focusNode: focusNode,
          autofocus: true,
          onPressed: null,
          icon: const Icon(Icons.link),
        ),
      ),
    );
    await tester.pump();
    expect(focusNode.hasPrimaryFocus, isFalse);

    focusNode.dispose();
  });

  testWidgets('IconButton keeps focus when disabled in directional navigation mode.', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'IconButton');
    await tester.pumpWidget(
      wrap(
        useMaterial3: theme.useMaterial3,
        child: MediaQuery(
          data: const MediaQueryData(navigationMode: NavigationMode.directional),
          child: IconButton(
            focusNode: focusNode,
            autofocus: true,
            onPressed: () {},
            icon: const Icon(Icons.link),
          ),
        ),
      ),
    );

    await tester.pump();
    expect(focusNode.hasPrimaryFocus, isTrue);

    await tester.pumpWidget(
      wrap(
        useMaterial3: theme.useMaterial3,
        child: MediaQuery(
          data: const MediaQueryData(navigationMode: NavigationMode.directional),
          child: IconButton(
            focusNode: focusNode,
            autofocus: true,
            onPressed: null,
            icon: const Icon(Icons.link),
          ),
        ),
      ),
    );
    await tester.pump();
    expect(focusNode.hasPrimaryFocus, isTrue);

    focusNode.dispose();
  });

  testWidgets("Disabled IconButton can't be traversed to when disabled.", (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode1 = FocusNode(debugLabel: 'IconButton 1');
    final FocusNode focusNode2 = FocusNode(debugLabel: 'IconButton 2');
    addTearDown(() {
      focusNode1.dispose();
      focusNode2.dispose();
    });

    await tester.pumpWidget(
      wrap(
        useMaterial3: theme.useMaterial3,
        child: Column(
          children: <Widget>[
            IconButton(
              focusNode: focusNode1,
              autofocus: true,
              onPressed: () {},
              icon: const Icon(Icons.link),
            ),
            IconButton(focusNode: focusNode2, onPressed: null, icon: const Icon(Icons.link)),
          ],
        ),
      ),
    );
    await tester.pump();

    expect(focusNode1.hasPrimaryFocus, isTrue);
    expect(focusNode2.hasPrimaryFocus, isFalse);

    expect(focusNode1.nextFocus(), isFalse);
    await tester.pump();

    expect(focusNode1.hasPrimaryFocus, !kIsWeb);
    expect(focusNode2.hasPrimaryFocus, isFalse);
  });

  group('feedback', () {
    late FeedbackTester feedback;

    setUp(() {
      feedback = FeedbackTester();
    });

    tearDown(() {
      feedback.dispose();
    });

    testWidgets('IconButton with disabled feedback', (WidgetTester tester) async {
      final Widget button = Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: IconButton(onPressed: () {}, enableFeedback: false, icon: const Icon(Icons.link)),
        ),
      );

      await tester.pumpWidget(
        theme.useMaterial3 ? MaterialApp(theme: theme, home: button) : Material(child: button),
      );
      await tester.tap(find.byType(IconButton), pointer: 1);
      await tester.pump(const Duration(seconds: 1));
      expect(feedback.clickSoundCount, 0);
      expect(feedback.hapticCount, 0);
    });

    testWidgets('IconButton with enabled feedback', (WidgetTester tester) async {
      final Widget button = Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: IconButton(onPressed: () {}, icon: const Icon(Icons.link))),
      );

      await tester.pumpWidget(
        theme.useMaterial3 ? MaterialApp(theme: theme, home: button) : Material(child: button),
      );
      await tester.tap(find.byType(IconButton), pointer: 1);
      await tester.pump(const Duration(seconds: 1));
      expect(feedback.clickSoundCount, 1);
      expect(feedback.hapticCount, 0);
    });

    testWidgets('IconButton with enabled feedback by default', (WidgetTester tester) async {
      final Widget button = Directionality(
        textDirection: TextDirection.ltr,
        child: Center(child: IconButton(onPressed: () {}, icon: const Icon(Icons.link))),
      );

      await tester.pumpWidget(
        theme.useMaterial3 ? MaterialApp(theme: theme, home: button) : Material(child: button),
      );
      await tester.tap(find.byType(IconButton), pointer: 1);
      await tester.pump(const Duration(seconds: 1));
      expect(feedback.clickSoundCount, 1);
      expect(feedback.hapticCount, 0);
    });
  });

  testWidgets('IconButton responds to density changes.', (WidgetTester tester) async {
    const Key key = Key('test');
    final bool material3 = theme.useMaterial3;
    Future<void> buildTest(VisualDensity visualDensity) async {
      return tester.pumpWidget(
        MaterialApp(
          theme: theme,
          home: Material(
            child: Center(
              child: IconButton(
                visualDensity: visualDensity,
                key: key,
                onPressed: () {},
                icon: const Icon(Icons.play_arrow),
              ),
            ),
          ),
        ),
      );
    }

    await buildTest(VisualDensity.standard);
    final RenderBox box = tester.renderObject(find.byType(IconButton));
    await tester.pumpAndSettle();
    expect(box.size, equals(const Size(48, 48)));

    await buildTest(const VisualDensity(horizontal: 3.0, vertical: 3.0));
    await tester.pumpAndSettle();
    expect(box.size, equals(material3 ? const Size(64, 64) : const Size(60, 60)));

    await buildTest(const VisualDensity(horizontal: -3.0, vertical: -3.0));
    await tester.pumpAndSettle();
    // IconButton is a subclass of ButtonStyleButton in Material 3, so the negative
    // visualDensity cannot be applied to horizontal padding.
    // The size of the Button with padding is (24 + 8 + 8, 24) -> (40, 24)
    // minSize of M3 IconButton is (48 - 12, 48 - 12) -> (36, 36)
    // So, the button size in Material 3 is (40, 36)
    expect(box.size, equals(material3 ? const Size(40, 36) : const Size(40, 40)));

    await buildTest(const VisualDensity(horizontal: 3.0, vertical: -3.0));
    await tester.pumpAndSettle();
    expect(box.size, equals(material3 ? const Size(64, 36) : const Size(60, 40)));
  });

  testWidgets('IconButton.mouseCursor changes cursor on hover', (WidgetTester tester) async {
    // Test argument works
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Center(
              child: IconButton(
                onPressed: () {},
                mouseCursor: SystemMouseCursors.forbidden,
                icon: const Icon(Icons.play_arrow),
              ),
            ),
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.byType(IconButton)));

    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.forbidden,
    );

    // Test default is click
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Center(child: IconButton(onPressed: () {}, icon: const Icon(Icons.play_arrow))),
          ),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );
  });

  testWidgets('disabled IconButton has basic mouse cursor', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Material(
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Center(
              child: IconButton(
                onPressed: null, // null value indicates IconButton is disabled
                icon: Icon(Icons.play_arrow),
              ),
            ),
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.byType(IconButton)));

    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );
  });

  testWidgets('IconButton.mouseCursor overrides implicit setting of mouse cursor', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Material(
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Center(
              child: IconButton(
                onPressed: null,
                mouseCursor: SystemMouseCursors.none,
                icon: Icon(Icons.play_arrow),
              ),
            ),
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.byType(IconButton)));

    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.none,
    );

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Material(
          child: Directionality(
            textDirection: TextDirection.ltr,
            child: Center(
              child: IconButton(
                onPressed: () {},
                mouseCursor: SystemMouseCursors.none,
                icon: const Icon(Icons.play_arrow),
              ),
            ),
          ),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.none,
    );
  });

  testWidgets('IconTheme opacity test', (WidgetTester tester) async {
    final ThemeData theme = ThemeData.from(colorScheme: colorScheme, useMaterial3: false);

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Center(
            child: IconButton(icon: const Icon(Icons.add), color: Colors.purple, onPressed: () {}),
          ),
        ),
      ),
    );

    Color? iconColor() => _iconStyle(tester, Icons.add)?.color;
    expect(iconColor(), Colors.purple);

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Center(
            child: IconTheme.merge(
              data: const IconThemeData(opacity: 0.5),
              child: IconButton(
                icon: const Icon(Icons.add),
                color: Colors.purple,
                onPressed: () {},
              ),
            ),
          ),
        ),
      ),
    );

    Color? iconColorWithOpacity() => _iconStyle(tester, Icons.add)?.color;
    expect(iconColorWithOpacity(), Colors.purple.withOpacity(0.5));
  });

  testWidgets('IconButton defaults - M3', (WidgetTester tester) async {
    final ThemeData themeM3 = ThemeData.from(colorScheme: colorScheme);

    // Enabled IconButton
    await tester.pumpWidget(
      MaterialApp(
        theme: themeM3,
        home: Center(child: IconButton(onPressed: () {}, icon: const Icon(Icons.ac_unit))),
      ),
    );

    final Finder buttonMaterial = find.descendant(
      of: find.byType(IconButton),
      matching: find.byType(Material),
    );

    Material material = tester.widget<Material>(buttonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, Colors.transparent);
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);

    final Align align = tester.firstWidget<Align>(
      find.ancestor(of: find.byIcon(Icons.ac_unit), matching: find.byType(Align)),
    );
    expect(align.alignment, Alignment.center);
    expect(tester.getSize(find.byIcon(Icons.ac_unit)), const Size(24.0, 24.0));

    final Offset center = tester.getCenter(find.byType(IconButton));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pump(); // start the splash animation
    await tester.pump(const Duration(milliseconds: 100)); // splash is underway

    await gesture.up();
    await tester.pumpAndSettle();
    material = tester.widget<Material>(buttonMaterial);
    // No change vs enabled and not pressed.
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, Colors.transparent);
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);

    // Disabled IconButton
    await tester.pumpWidget(
      MaterialApp(
        theme: themeM3,
        home: const Center(child: IconButton(onPressed: null, icon: Icon(Icons.ac_unit))),
      ),
    );

    material = tester.widget<Material>(buttonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, Colors.transparent);
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);
  });

  testWidgets('IconButton default overlayColor resolves pressed state', (
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
                return IconButton(
                  onPressed: () {},
                  focusNode: focusNode,
                  icon: const Icon(Icons.add),
                );
              },
            ),
          ),
        ),
      ),
    );

    // Hovered.
    final Offset center = tester.getCenter(find.byType(IconButton));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(
      getOverlayColor(tester),
      paints..rect(color: theme.colorScheme.onSurfaceVariant.withOpacity(0.08)),
    );

    // Highlighted (pressed).
    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(
      getOverlayColor(tester),
      paints
        ..rect()
        ..rect(color: theme.colorScheme.onSurfaceVariant.withOpacity(0.1)),
    );
    // Remove pressed and hovered states
    await gesture.up();
    await tester.pumpAndSettle();
    await gesture.moveTo(const Offset(0, 50));
    await tester.pumpAndSettle();

    // Focused.
    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(
      getOverlayColor(tester),
      paints..rect(color: theme.colorScheme.onSurfaceVariant.withOpacity(0.1)),
    );

    focusNode.dispose();
  });

  testWidgets('IconButton.fill defaults - M3', (WidgetTester tester) async {
    final ThemeData themeM3 = ThemeData.from(colorScheme: colorScheme);

    // Enabled IconButton
    await tester.pumpWidget(
      MaterialApp(
        theme: themeM3,
        home: Center(child: IconButton.filled(onPressed: () {}, icon: const Icon(Icons.ac_unit))),
      ),
    );

    final Finder buttonMaterial = find.descendant(
      of: find.byType(IconButton),
      matching: find.byType(Material),
    );
    Color? iconColor() => _iconStyle(tester, Icons.ac_unit)?.color;
    expect(iconColor(), colorScheme.onPrimary);

    Material material = tester.widget<Material>(buttonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.primary);
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);

    final Align align = tester.firstWidget<Align>(
      find.ancestor(of: find.byIcon(Icons.ac_unit), matching: find.byType(Align)),
    );
    expect(align.alignment, Alignment.center);
    expect(tester.getSize(find.byIcon(Icons.ac_unit)), const Size(24.0, 24.0));

    final Offset center = tester.getCenter(find.byType(IconButton));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pump(); // start the splash animation
    await tester.pump(const Duration(milliseconds: 100)); // splash is underway

    await gesture.up();
    await tester.pumpAndSettle();
    material = tester.widget<Material>(buttonMaterial);
    // No change vs enabled and not pressed.
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.primary);
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);

    // Disabled IconButton
    await tester.pumpWidget(
      MaterialApp(
        theme: themeM3,
        home: const Center(child: IconButton.filled(onPressed: null, icon: Icon(Icons.ac_unit))),
      ),
    );

    material = tester.widget<Material>(buttonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.onSurface.withOpacity(0.12));
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);
    expect(iconColor(), colorScheme.onSurface.withOpacity(0.38));
  });

  testWidgets('IconButton.fill default overlayColor resolves pressed state', (
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
                return IconButton.filled(
                  onPressed: () {},
                  focusNode: focusNode,
                  icon: const Icon(Icons.add),
                );
              },
            ),
          ),
        ),
      ),
    );

    // Hovered.
    final Offset center = tester.getCenter(find.byType(IconButton));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(
      getOverlayColor(tester),
      paints..rect(color: theme.colorScheme.onPrimary.withOpacity(0.08)),
    );

    // Highlighted (pressed).
    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(
      getOverlayColor(tester),
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
    expect(
      getOverlayColor(tester),
      paints..rect(color: theme.colorScheme.onPrimary.withOpacity(0.1)),
    );

    focusNode.dispose();
  });

  testWidgets('Toggleable IconButton.fill defaults - M3', (WidgetTester tester) async {
    final ThemeData themeM3 = ThemeData.from(colorScheme: colorScheme);

    // Enabled selected IconButton
    await tester.pumpWidget(
      MaterialApp(
        theme: themeM3,
        home: Center(
          child: IconButton.filled(
            isSelected: true,
            onPressed: () {},
            icon: const Icon(Icons.ac_unit),
          ),
        ),
      ),
    );

    final Finder buttonMaterial = find.descendant(
      of: find.byType(IconButton),
      matching: find.byType(Material),
    );
    Color? iconColor() => _iconStyle(tester, Icons.ac_unit)?.color;
    expect(iconColor(), colorScheme.onPrimary);

    Material material = tester.widget<Material>(buttonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.primary);
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);

    final Align align = tester.firstWidget<Align>(
      find.ancestor(of: find.byIcon(Icons.ac_unit), matching: find.byType(Align)),
    );
    expect(align.alignment, Alignment.center);
    expect(tester.getSize(find.byIcon(Icons.ac_unit)), const Size(24.0, 24.0));

    final Offset center = tester.getCenter(find.byType(IconButton));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pump(); // start the splash animation
    await tester.pump(const Duration(milliseconds: 100)); // splash is underway

    await gesture.up();
    await tester.pumpAndSettle();
    material = tester.widget<Material>(buttonMaterial);
    // No change vs enabled and not pressed.
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.primary);
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);

    // Enabled unselected IconButton
    await tester.pumpWidget(
      MaterialApp(
        theme: themeM3,
        home: Center(
          child: IconButton.filled(
            isSelected: false,
            onPressed: () {},
            icon: const Icon(Icons.ac_unit),
          ),
        ),
      ),
    );

    material = tester.widget<Material>(buttonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.surfaceVariant);
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);
    expect(iconColor(), colorScheme.primary);

    // Disabled IconButton
    await tester.pumpWidget(
      MaterialApp(
        theme: themeM3,
        home: const Center(
          child: IconButton.filled(isSelected: true, onPressed: null, icon: Icon(Icons.ac_unit)),
        ),
      ),
    );

    material = tester.widget<Material>(buttonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.onSurface.withOpacity(0.12));
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);
    expect(iconColor(), colorScheme.onSurface.withOpacity(0.38));
  });

  testWidgets('IconButton.filledTonal defaults - M3', (WidgetTester tester) async {
    final ThemeData themeM3 = ThemeData.from(colorScheme: colorScheme);

    // Enabled IconButton.tonal
    await tester.pumpWidget(
      MaterialApp(
        theme: themeM3,
        home: Center(
          child: IconButton.filledTonal(onPressed: () {}, icon: const Icon(Icons.ac_unit)),
        ),
      ),
    );

    final Finder buttonMaterial = find.descendant(
      of: find.byType(IconButton),
      matching: find.byType(Material),
    );
    Color? iconColor() => _iconStyle(tester, Icons.ac_unit)?.color;
    expect(iconColor(), colorScheme.onSecondaryContainer);

    Material material = tester.widget<Material>(buttonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.secondaryContainer);
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);

    final Align align = tester.firstWidget<Align>(
      find.ancestor(of: find.byIcon(Icons.ac_unit), matching: find.byType(Align)),
    );
    expect(align.alignment, Alignment.center);
    expect(tester.getSize(find.byIcon(Icons.ac_unit)), const Size(24.0, 24.0));

    final Offset center = tester.getCenter(find.byType(IconButton));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pump(); // start the splash animation
    await tester.pump(const Duration(milliseconds: 100)); // splash is underway

    await gesture.up();
    await tester.pumpAndSettle();
    material = tester.widget<Material>(buttonMaterial);
    // No change vs enabled and not pressed.
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.secondaryContainer);
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);

    // Disabled IconButton
    await tester.pumpWidget(
      MaterialApp(
        theme: themeM3,
        home: const Center(
          child: IconButton.filledTonal(onPressed: null, icon: Icon(Icons.ac_unit)),
        ),
      ),
    );

    material = tester.widget<Material>(buttonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.onSurface.withOpacity(0.12));
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);
    expect(iconColor(), colorScheme.onSurface.withOpacity(0.38));
  });

  testWidgets('IconButton.filledTonal default overlayColor resolves pressed state', (
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
                return IconButton.filledTonal(
                  onPressed: () {},
                  focusNode: focusNode,
                  icon: const Icon(Icons.add),
                );
              },
            ),
          ),
        ),
      ),
    );

    // Hovered.
    final Offset center = tester.getCenter(find.byType(IconButton));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(
      getOverlayColor(tester),
      paints..rect(color: theme.colorScheme.onSecondaryContainer.withOpacity(0.08)),
    );

    // Highlighted (pressed).
    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(
      getOverlayColor(tester),
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
    expect(
      getOverlayColor(tester),
      paints..rect(color: theme.colorScheme.onSecondaryContainer.withOpacity(0.1)),
    );

    focusNode.dispose();
  });

  testWidgets('Toggleable IconButton.filledTonal defaults - M3', (WidgetTester tester) async {
    final ThemeData themeM3 = ThemeData.from(colorScheme: colorScheme);

    // Enabled selected IconButton
    await tester.pumpWidget(
      MaterialApp(
        theme: themeM3,
        home: Center(
          child: IconButton.filledTonal(
            isSelected: true,
            onPressed: () {},
            icon: const Icon(Icons.ac_unit),
          ),
        ),
      ),
    );

    final Finder buttonMaterial = find.descendant(
      of: find.byType(IconButton),
      matching: find.byType(Material),
    );
    Color? iconColor() => _iconStyle(tester, Icons.ac_unit)?.color;
    expect(iconColor(), colorScheme.onSecondaryContainer);

    Material material = tester.widget<Material>(buttonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.secondaryContainer);
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);

    final Align align = tester.firstWidget<Align>(
      find.ancestor(of: find.byIcon(Icons.ac_unit), matching: find.byType(Align)),
    );
    expect(align.alignment, Alignment.center);
    expect(tester.getSize(find.byIcon(Icons.ac_unit)), const Size(24.0, 24.0));

    final Offset center = tester.getCenter(find.byType(IconButton));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pump(); // start the splash animation
    await tester.pump(const Duration(milliseconds: 100)); // splash is underway

    await gesture.up();
    await tester.pumpAndSettle();
    material = tester.widget<Material>(buttonMaterial);
    // No change vs enabled and not pressed.
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.secondaryContainer);
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);

    // Enabled unselected IconButton
    await tester.pumpWidget(
      MaterialApp(
        theme: themeM3,
        home: Center(
          child: IconButton.filledTonal(
            isSelected: false,
            onPressed: () {},
            icon: const Icon(Icons.ac_unit),
          ),
        ),
      ),
    );

    material = tester.widget<Material>(buttonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.surfaceVariant);
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);
    expect(iconColor(), colorScheme.onSurfaceVariant);

    // Disabled IconButton
    await tester.pumpWidget(
      MaterialApp(
        theme: themeM3,
        home: const Center(
          child: IconButton.filledTonal(
            isSelected: true,
            onPressed: null,
            icon: Icon(Icons.ac_unit),
          ),
        ),
      ),
    );

    material = tester.widget<Material>(buttonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.onSurface.withOpacity(0.12));
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);
    expect(iconColor(), colorScheme.onSurface.withOpacity(0.38));
  });

  testWidgets('IconButton.outlined defaults - M3', (WidgetTester tester) async {
    final ThemeData themeM3 = ThemeData.from(colorScheme: colorScheme);

    // Enabled IconButton.tonal
    await tester.pumpWidget(
      MaterialApp(
        theme: themeM3,
        home: Center(child: IconButton.outlined(onPressed: () {}, icon: const Icon(Icons.ac_unit))),
      ),
    );

    final Finder buttonMaterial = find.descendant(
      of: find.byType(IconButton),
      matching: find.byType(Material),
    );
    Color? iconColor() => _iconStyle(tester, Icons.ac_unit)?.color;
    expect(iconColor(), colorScheme.onSurfaceVariant);

    Material material = tester.widget<Material>(buttonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, Colors.transparent);
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, StadiumBorder(side: BorderSide(color: colorScheme.outline)));
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);

    final Align align = tester.firstWidget<Align>(
      find.ancestor(of: find.byIcon(Icons.ac_unit), matching: find.byType(Align)),
    );
    expect(align.alignment, Alignment.center);
    expect(tester.getSize(find.byIcon(Icons.ac_unit)), const Size(24.0, 24.0));

    final Offset center = tester.getCenter(find.byType(IconButton));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pump(); // start the splash animation
    await tester.pump(const Duration(milliseconds: 100)); // splash is underway

    await gesture.up();
    await tester.pumpAndSettle();
    material = tester.widget<Material>(buttonMaterial);
    // No change vs enabled and not pressed.
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, Colors.transparent);
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, StadiumBorder(side: BorderSide(color: colorScheme.outline)));
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);

    // Disabled IconButton
    await tester.pumpWidget(
      MaterialApp(
        theme: themeM3,
        home: const Center(child: IconButton.outlined(onPressed: null, icon: Icon(Icons.ac_unit))),
      ),
    );

    material = tester.widget<Material>(buttonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, Colors.transparent);
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(
      material.shape,
      StadiumBorder(side: BorderSide(color: colorScheme.onSurface.withOpacity(0.12))),
    );
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);
    expect(iconColor(), colorScheme.onSurface.withOpacity(0.38));
  });

  testWidgets('IconButton.outlined default overlayColor resolves pressed state', (
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
                return IconButton.outlined(
                  onPressed: () {},
                  focusNode: focusNode,
                  icon: const Icon(Icons.add),
                );
              },
            ),
          ),
        ),
      ),
    );

    // Hovered.
    final Offset center = tester.getCenter(find.byType(IconButton));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(
      getOverlayColor(tester),
      paints..rect(color: theme.colorScheme.onSurfaceVariant.withOpacity(0.08)),
    );

    // Highlighted (pressed).
    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(
      getOverlayColor(tester),
      paints
        ..rect()
        ..rect(color: theme.colorScheme.onSurface.withOpacity(0.1)),
    );
    // Remove pressed and hovered states
    await gesture.up();
    await tester.pumpAndSettle();
    await gesture.moveTo(const Offset(0, 50));
    await tester.pumpAndSettle();

    // Focused.
    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(
      getOverlayColor(tester),
      paints..rect(color: theme.colorScheme.onSurfaceVariant.withOpacity(0.08)),
    );

    focusNode.dispose();
  });

  testWidgets('Toggleable IconButton.outlined defaults - M3', (WidgetTester tester) async {
    final ThemeData themeM3 = ThemeData.from(colorScheme: colorScheme);

    // Enabled selected IconButton
    await tester.pumpWidget(
      MaterialApp(
        theme: themeM3,
        home: Center(
          child: IconButton.outlined(
            isSelected: true,
            onPressed: () {},
            icon: const Icon(Icons.ac_unit),
          ),
        ),
      ),
    );

    final Finder buttonMaterial = find.descendant(
      of: find.byType(IconButton),
      matching: find.byType(Material),
    );
    Color? iconColor() => _iconStyle(tester, Icons.ac_unit)?.color;
    expect(iconColor(), colorScheme.onInverseSurface);

    Material material = tester.widget<Material>(buttonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.inverseSurface);
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);

    final Align align = tester.firstWidget<Align>(
      find.ancestor(of: find.byIcon(Icons.ac_unit), matching: find.byType(Align)),
    );
    expect(align.alignment, Alignment.center);
    expect(tester.getSize(find.byIcon(Icons.ac_unit)), const Size(24.0, 24.0));

    final Offset center = tester.getCenter(find.byType(IconButton));
    final TestGesture gesture = await tester.startGesture(center);
    await tester.pump(); // start the splash animation
    await tester.pump(const Duration(milliseconds: 100)); // splash is underway

    await gesture.up();
    await tester.pumpAndSettle();
    material = tester.widget<Material>(buttonMaterial);
    // No change vs enabled and not pressed.
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.inverseSurface);
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);

    // Enabled unselected IconButton
    await tester.pumpWidget(
      MaterialApp(
        theme: themeM3,
        home: Center(
          child: IconButton.outlined(
            isSelected: false,
            onPressed: () {},
            icon: const Icon(Icons.ac_unit),
          ),
        ),
      ),
    );

    material = tester.widget<Material>(buttonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, Colors.transparent);
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, StadiumBorder(side: BorderSide(color: colorScheme.outline)));
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);
    expect(iconColor(), colorScheme.onSurfaceVariant);

    // Disabled IconButton
    await tester.pumpWidget(
      MaterialApp(
        theme: themeM3,
        home: const Center(
          child: IconButton.outlined(isSelected: true, onPressed: null, icon: Icon(Icons.ac_unit)),
        ),
      ),
    );

    material = tester.widget<Material>(buttonMaterial);
    expect(material.animationDuration, const Duration(milliseconds: 200));
    expect(material.borderOnForeground, true);
    expect(material.borderRadius, null);
    expect(material.clipBehavior, Clip.none);
    expect(material.color, colorScheme.onSurface.withOpacity(0.12));
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);
    expect(material.shape, const StadiumBorder());
    expect(material.textStyle, null);
    expect(material.type, MaterialType.button);
    expect(iconColor(), colorScheme.onSurface.withOpacity(0.38));
  });

  testWidgets(
    'Default IconButton meets a11y contrast guidelines - M3',
    (WidgetTester tester) async {
      final FocusNode focusNode = FocusNode();

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData.from(colorScheme: const ColorScheme.light()),
          home: Scaffold(
            body: Center(
              child: IconButton(
                onPressed: () {},
                focusNode: focusNode,
                icon: const Icon(Icons.ac_unit),
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
      final Offset center = tester.getCenter(find.byType(IconButton));
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer();
      await gesture.moveTo(center);
      await tester.pumpAndSettle();
      await expectLater(tester, meetsGuideline(textContrastGuideline));

      // Highlighted (pressed).
      await gesture.down(center);
      await tester.pump(); // Start the splash and highlight animations.
      await tester.pump(
        const Duration(milliseconds: 800),
      ); // Wait for splash and highlight to be well under way.
      await expectLater(tester, meetsGuideline(textContrastGuideline));

      await gesture.removePointer();

      focusNode.dispose();
    },
    skip: isBrowser, // https://github.com/flutter/flutter/issues/44115
  );

  testWidgets('IconButton uses stateful color for icon color in different states - M3', (
    WidgetTester tester,
  ) async {
    bool isSelected = false;
    final FocusNode focusNode = FocusNode();

    const Color pressedColor = Color(0x00000001);
    const Color hoverColor = Color(0x00000002);
    const Color focusedColor = Color(0x00000003);
    const Color defaultColor = Color(0x00000004);
    const Color selectedColor = Color(0x00000005);

    Color getIconColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.pressed)) {
        return pressedColor;
      }
      if (states.contains(MaterialState.hovered)) {
        return hoverColor;
      }
      if (states.contains(MaterialState.focused)) {
        return focusedColor;
      }
      if (states.contains(MaterialState.selected)) {
        return selectedColor;
      }
      return defaultColor;
    }

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.from(colorScheme: const ColorScheme.light()),
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Scaffold(
              body: Center(
                child: IconButton(
                  style: ButtonStyle(
                    foregroundColor: MaterialStateProperty.resolveWith<Color>(getIconColor),
                  ),
                  isSelected: isSelected,
                  onPressed: () {
                    setState(() {
                      isSelected = !isSelected;
                    });
                  },
                  focusNode: focusNode,
                  icon: const Icon(Icons.ac_unit),
                ),
              ),
            );
          },
        ),
      ),
    );

    Color? iconColor() => _iconStyle(tester, Icons.ac_unit)?.color;

    // Default, not disabled.
    expect(iconColor(), equals(defaultColor));

    // Selected
    final Finder button = find.byType(IconButton);
    await tester.tap(button);
    await tester.pumpAndSettle();
    expect(iconColor(), selectedColor);

    // Focused.
    focusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(iconColor(), focusedColor);

    // Hovered.
    final Offset center = tester.getCenter(find.byType(IconButton));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(iconColor(), hoverColor);

    // Highlighted (pressed).
    await gesture.down(center);
    await tester.pump(); // Start the splash and highlight animations.
    await tester.pump(
      const Duration(milliseconds: 800),
    ); // Wait for splash and highlight to be well under way.
    expect(iconColor(), pressedColor);

    focusNode.dispose();
  });

  testWidgets('Does IconButton contribute semantics - M3', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Center(
          child: Theme(
            data: ThemeData(),
            child: IconButton(
              style: const ButtonStyle(
                // Specifying minimumSize to mimic the original minimumSize for
                // RaisedButton so that the semantics tree's rect and transform
                // match the original version of this test.
                minimumSize: MaterialStatePropertyAll<Size>(Size(88, 36)),
              ),
              onPressed: () {},
              icon: const Icon(Icons.ac_unit),
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

  testWidgets('IconButton size is configurable by ThemeData.materialTapTargetSize - M3', (
    WidgetTester tester,
  ) async {
    Widget buildFrame(MaterialTapTargetSize tapTargetSize) {
      return Theme(
        data: ThemeData(materialTapTargetSize: tapTargetSize),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Center(
            child: IconButton(
              style: IconButton.styleFrom(minimumSize: const Size(40, 40)),
              icon: const Icon(Icons.ac_unit),
              onPressed: () {},
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(MaterialTapTargetSize.padded));
    expect(tester.getSize(find.byType(IconButton)), const Size(48.0, 48.0));

    await tester.pumpWidget(buildFrame(MaterialTapTargetSize.shrinkWrap));
    expect(tester.getSize(find.byType(IconButton)), const Size(40.0, 40.0));
  });

  testWidgets('Override IconButton default padding - M3', (WidgetTester tester) async {
    // Use [IconButton]'s padding property to override default value.
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.from(colorScheme: const ColorScheme.light()),
        home: Scaffold(
          body: Center(
            child: IconButton(
              padding: const EdgeInsets.all(20),
              onPressed: () {},
              icon: const Icon(Icons.ac_unit),
            ),
          ),
        ),
      ),
    );

    final Padding paddingWidget1 = tester.widget<Padding>(
      find.descendant(of: find.byType(IconButton), matching: find.byType(Padding)),
    );
    expect(paddingWidget1.padding, const EdgeInsets.all(20));

    // Use [IconButton.style]'s padding property to override default value.
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.from(colorScheme: const ColorScheme.light()),
        home: Scaffold(
          body: Center(
            child: IconButton(
              style: IconButton.styleFrom(padding: const EdgeInsets.all(20)),
              onPressed: () {},
              icon: const Icon(Icons.ac_unit),
            ),
          ),
        ),
      ),
    );

    final Padding paddingWidget2 = tester.widget<Padding>(
      find.descendant(of: find.byType(IconButton), matching: find.byType(Padding)),
    );
    expect(paddingWidget2.padding, const EdgeInsets.all(20));

    // [IconButton.style]'s padding will override [IconButton]'s padding if both
    // values are not null.
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.from(colorScheme: const ColorScheme.light()),
        home: Scaffold(
          body: Center(
            child: IconButton(
              padding: const EdgeInsets.all(15),
              style: IconButton.styleFrom(padding: const EdgeInsets.all(22)),
              onPressed: () {},
              icon: const Icon(Icons.ac_unit),
            ),
          ),
        ),
      ),
    );

    final Padding paddingWidget3 = tester.widget<Padding>(
      find.descendant(of: find.byType(IconButton), matching: find.byType(Padding)),
    );
    expect(paddingWidget3.padding, const EdgeInsets.all(22));
  });

  testWidgets('Default IconButton is not selectable - M3', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.from(colorScheme: const ColorScheme.light()),
        home: IconButton(icon: const Icon(Icons.ac_unit), onPressed: () {}),
      ),
    );

    final Finder button = find.byType(IconButton);
    IconButton buttonWidget() => tester.widget<IconButton>(button);

    Material buttonMaterial() {
      return tester.widget<Material>(
        find.descendant(of: find.byType(IconButton), matching: find.byType(Material)),
      );
    }

    Color? iconColor() => _iconStyle(tester, Icons.ac_unit)?.color;

    expect(buttonWidget().isSelected, null);
    expect(iconColor(), equals(const ColorScheme.light().onSurfaceVariant));
    expect(buttonMaterial().color, Colors.transparent);

    await tester.tap(
      button,
    ); // The non-toggle IconButton should not change appearance after clicking
    await tester.pumpAndSettle();

    expect(buttonWidget().isSelected, null);
    expect(iconColor(), equals(const ColorScheme.light().onSurfaceVariant));
    expect(buttonMaterial().color, Colors.transparent);
  });

  testWidgets('Icon button is selectable when isSelected is not null - M3', (
    WidgetTester tester,
  ) async {
    bool isSelected = false;
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.from(colorScheme: const ColorScheme.light()),
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return IconButton(
              isSelected: isSelected,
              icon: const Icon(Icons.ac_unit),
              onPressed: () {
                setState(() {
                  isSelected = !isSelected;
                });
              },
            );
          },
        ),
      ),
    );

    final Finder button = find.byType(IconButton);
    IconButton buttonWidget() => tester.widget<IconButton>(button);
    Color? iconColor() => _iconStyle(tester, Icons.ac_unit)?.color;

    Material buttonMaterial() {
      return tester.widget<Material>(
        find.descendant(of: find.byType(IconButton), matching: find.byType(Material)),
      );
    }

    expect(buttonWidget().isSelected, false);
    expect(iconColor(), equals(const ColorScheme.light().onSurfaceVariant));
    expect(buttonMaterial().color, Colors.transparent);

    await tester.tap(button); // The toggle IconButton should change appearance after clicking
    await tester.pumpAndSettle();

    expect(buttonWidget().isSelected, true);
    expect(iconColor(), equals(const ColorScheme.light().primary));
    expect(buttonMaterial().color, Colors.transparent);

    await tester.tap(button); // The IconButton should be unselected if it's clicked again
    await tester.pumpAndSettle();

    expect(buttonWidget().isSelected, false);
    expect(iconColor(), equals(const ColorScheme.light().onSurfaceVariant));
    expect(buttonMaterial().color, Colors.transparent);
  });

  testWidgets('The IconButton is in selected status if isSelected is true by default - M3', (
    WidgetTester tester,
  ) async {
    bool isSelected = true;
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.from(colorScheme: const ColorScheme.light()),
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return IconButton(
              isSelected: isSelected,
              icon: const Icon(Icons.ac_unit),
              onPressed: () {
                setState(() {
                  isSelected = !isSelected;
                });
              },
            );
          },
        ),
      ),
    );

    final Finder button = find.byType(IconButton);
    IconButton buttonWidget() => tester.widget<IconButton>(button);
    Color? iconColor() => _iconStyle(tester, Icons.ac_unit)?.color;

    Material buttonMaterial() {
      return tester.widget<Material>(
        find.descendant(of: find.byType(IconButton), matching: find.byType(Material)),
      );
    }

    expect(buttonWidget().isSelected, true);
    expect(iconColor(), equals(const ColorScheme.light().primary));
    expect(buttonMaterial().color, Colors.transparent);

    await tester.tap(button); // The IconButton becomes unselected if it's clicked
    await tester.pumpAndSettle();

    expect(buttonWidget().isSelected, false);
    expect(iconColor(), equals(const ColorScheme.light().onSurfaceVariant));
    expect(buttonMaterial().color, Colors.transparent);
  });

  testWidgets("The selectedIcon is used if it's not null and the button is clicked", (
    WidgetTester tester,
  ) async {
    bool isSelected = false;
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.from(colorScheme: const ColorScheme.light()),
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return IconButton(
              isSelected: isSelected,
              selectedIcon: const Icon(Icons.account_box),
              icon: const Icon(Icons.account_box_outlined),
              onPressed: () {
                setState(() {
                  isSelected = !isSelected;
                });
              },
            );
          },
        ),
      ),
    );

    final Finder button = find.byType(IconButton);

    expect(find.byIcon(Icons.account_box_outlined), findsOneWidget);
    expect(find.byIcon(Icons.account_box), findsNothing);

    await tester.tap(button); // The icon becomes to selectedIcon
    await tester.pumpAndSettle();

    expect(find.byIcon(Icons.account_box), findsOneWidget);
    expect(find.byIcon(Icons.account_box_outlined), findsNothing);

    await tester.tap(button); // The icon becomes the original icon when it's clicked again
    await tester.pumpAndSettle();

    expect(find.byIcon(Icons.account_box_outlined), findsOneWidget);
    expect(find.byIcon(Icons.account_box), findsNothing);
  });

  testWidgets(
    'The original icon is used for selected and unselected status when selectedIcon is null',
    (WidgetTester tester) async {
      bool isSelected = false;
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData.from(colorScheme: const ColorScheme.light()),
          home: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return IconButton(
                isSelected: isSelected,
                icon: const Icon(Icons.account_box),
                onPressed: () {
                  setState(() {
                    isSelected = !isSelected;
                  });
                },
              );
            },
          ),
        ),
      );

      final Finder button = find.byType(IconButton);
      IconButton buttonWidget() => tester.widget<IconButton>(button);

      expect(buttonWidget().isSelected, false);
      expect(buttonWidget().selectedIcon, null);
      expect(find.byIcon(Icons.account_box), findsOneWidget);

      await tester.tap(button); // The icon becomes the original icon when it's clicked again
      await tester.pumpAndSettle();

      expect(buttonWidget().isSelected, true);
      expect(buttonWidget().selectedIcon, null);
      expect(find.byIcon(Icons.account_box), findsOneWidget);
    },
  );

  testWidgets('The selectedIcon is used for disabled button if isSelected is true - M3', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.from(colorScheme: const ColorScheme.light()),
        home: const IconButton(
          isSelected: true,
          icon: Icon(Icons.account_box),
          selectedIcon: Icon(Icons.ac_unit),
          onPressed: null,
        ),
      ),
    );

    final Finder button = find.byType(IconButton);
    IconButton buttonWidget() => tester.widget<IconButton>(button);

    expect(buttonWidget().isSelected, true);
    expect(find.byIcon(Icons.account_box), findsNothing);
    expect(find.byIcon(Icons.ac_unit), findsOneWidget);
  });

  testWidgets('The visualDensity of M3 IconButton can be configured by IconButtonTheme, '
      'but cannot be configured by ThemeData - M3', (WidgetTester tester) async {
    Future<void> buildTest({
      VisualDensity? iconButtonThemeVisualDensity,
      VisualDensity? themeVisualDensity,
    }) async {
      return tester.pumpWidget(
        MaterialApp(
          theme: ThemeData.from(colorScheme: colorScheme).copyWith(
            iconButtonTheme: IconButtonThemeData(
              style: IconButton.styleFrom(visualDensity: iconButtonThemeVisualDensity),
            ),
            visualDensity: themeVisualDensity,
          ),
          home: Material(
            child: Center(child: IconButton(onPressed: () {}, icon: const Icon(Icons.play_arrow))),
          ),
        ),
      );
    }

    await buildTest(iconButtonThemeVisualDensity: VisualDensity.standard);
    final RenderBox box = tester.renderObject(find.byType(IconButton));
    await tester.pumpAndSettle();
    expect(box.size, equals(const Size(48, 48)));

    await buildTest(iconButtonThemeVisualDensity: VisualDensity.compact);
    await tester.pumpAndSettle();
    expect(box.size, equals(const Size(40, 40)));

    await buildTest(
      iconButtonThemeVisualDensity: const VisualDensity(horizontal: 3.0, vertical: 3.0),
    );
    await tester.pumpAndSettle();
    expect(box.size, equals(const Size(64, 64)));

    // ThemeData.visualDensity will be ignored because useMaterial3 is true
    await buildTest(themeVisualDensity: VisualDensity.standard);
    await tester.pumpAndSettle();
    expect(box.size, equals(const Size(48, 48)));

    await buildTest(themeVisualDensity: VisualDensity.compact);
    await tester.pumpAndSettle();
    expect(box.size, equals(const Size(48, 48)));

    await buildTest(themeVisualDensity: const VisualDensity(horizontal: 3.0, vertical: 3.0));
    await tester.pumpAndSettle();
    expect(box.size, equals(const Size(48, 48)));
  });

  testWidgets('IconButton.styleFrom overlayColor overrides default overlay color', (
    WidgetTester tester,
  ) async {
    const Color overlayColor = Color(0xffff0000);
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: IconButton(
              style: IconButton.styleFrom(overlayColor: overlayColor),
              onPressed: () {},
              icon: const Icon(Icons.add),
            ),
          ),
        ),
      ),
    );

    // Hovered.
    final Offset center = tester.getCenter(find.byType(IconButton));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(getOverlayColor(tester), paints..rect(color: overlayColor.withOpacity(0.08)));

    // Highlighted (pressed).
    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(
      getOverlayColor(tester),
      paints
        ..rect(color: overlayColor.withOpacity(0.08))
        ..rect(color: overlayColor.withOpacity(0.1)),
    );
    // Remove pressed and hovered states,
    await gesture.up();
    await tester.pumpAndSettle();
    await gesture.moveTo(const Offset(0, 50));
    await tester.pumpAndSettle();

    // Focused.
    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.pumpAndSettle();
    expect(getOverlayColor(tester), paints..rect(color: overlayColor.withOpacity(0.1)));
  });

  testWidgets('IconButton.styleFrom highlight, hover, focus colors overrides overlayColor', (
    WidgetTester tester,
  ) async {
    const Color hoverColor = Color(0xff0000f2);
    const Color highlightColor = Color(0xff0000f1);
    const Color focusColor = Color(0xff0000f3);
    const Color overlayColor = Color(0xffff0000);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: IconButton(
              style: IconButton.styleFrom(
                hoverColor: hoverColor,
                highlightColor: highlightColor,
                focusColor: focusColor,
                overlayColor: overlayColor,
              ),
              onPressed: () {},
              icon: const Icon(Icons.add),
            ),
          ),
        ),
      ),
    );

    // Hovered.
    final Offset center = tester.getCenter(find.byType(IconButton));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(getOverlayColor(tester), paints..rect(color: hoverColor));

    // Highlighted (pressed).
    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(
      getOverlayColor(tester),
      paints
        ..rect(color: hoverColor)
        ..rect(color: highlightColor),
    );
    // Remove pressed and hovered states,
    await gesture.up();
    await tester.pumpAndSettle();
    await gesture.moveTo(const Offset(0, 50));
    await tester.pumpAndSettle();

    // Focused.
    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.pumpAndSettle();
    expect(getOverlayColor(tester), paints..rect(color: focusColor));
  });

  testWidgets('IconButton.styleFrom with transparent overlayColor', (WidgetTester tester) async {
    const Color overlayColor = Colors.transparent;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: IconButton(
              style: IconButton.styleFrom(overlayColor: overlayColor),
              onPressed: () {},
              icon: const Icon(Icons.add),
            ),
          ),
        ),
      ),
    );

    // Hovered.
    final Offset center = tester.getCenter(find.byType(IconButton));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(getOverlayColor(tester), paints..rect(color: overlayColor));

    // Highlighted (pressed).
    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(
      getOverlayColor(tester),
      paints
        ..rect(color: overlayColor)
        ..rect(color: overlayColor),
    );
    // Remove pressed and hovered states,
    await gesture.up();
    await tester.pumpAndSettle();
    await gesture.moveTo(const Offset(0, 50));
    await tester.pumpAndSettle();

    // Focused.
    await tester.sendKeyEvent(LogicalKeyboardKey.tab);
    await tester.pumpAndSettle();
    expect(getOverlayColor(tester), paints..rect(color: overlayColor));
  });

  group('IconTheme tests in Material 3', () {
    testWidgets('IconTheme overrides default values in M3', (WidgetTester tester) async {
      // Theme's IconTheme
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData.from(
            colorScheme: const ColorScheme.light(),
          ).copyWith(iconTheme: const IconThemeData(color: Colors.red, size: 37)),
          home: IconButton(icon: const Icon(Icons.account_box), onPressed: () {}),
        ),
      );

      Color? iconColor0() => _iconStyle(tester, Icons.account_box)?.color;
      expect(iconColor0(), Colors.red);
      expect(tester.getSize(find.byIcon(Icons.account_box)), equals(const Size(37, 37)));

      // custom IconTheme outside of IconButton
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData.from(colorScheme: const ColorScheme.light()),
          home: IconTheme.merge(
            data: const IconThemeData(color: Colors.pink, size: 35),
            child: IconButton(icon: const Icon(Icons.account_box), onPressed: () {}),
          ),
        ),
      );

      Color? iconColor1() => _iconStyle(tester, Icons.account_box)?.color;
      expect(iconColor1(), Colors.pink);
      expect(tester.getSize(find.byIcon(Icons.account_box)), equals(const Size(35, 35)));
    });

    testWidgets('Theme IconButtonTheme overrides IconTheme in Material3', (
      WidgetTester tester,
    ) async {
      // When IconButtonTheme and IconTheme both exist in ThemeData, the IconButtonTheme can override IconTheme.
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData.from(colorScheme: const ColorScheme.light()).copyWith(
            iconTheme: const IconThemeData(color: Colors.red, size: 25),
            iconButtonTheme: IconButtonThemeData(
              style: IconButton.styleFrom(foregroundColor: Colors.green, iconSize: 27),
            ),
          ),
          home: IconButton(icon: const Icon(Icons.account_box), onPressed: () {}),
        ),
      );

      Color? iconColor() => _iconStyle(tester, Icons.account_box)?.color;
      expect(iconColor(), Colors.green);
      expect(tester.getSize(find.byIcon(Icons.account_box)), equals(const Size(27, 27)));
    });

    testWidgets('Button IconButtonTheme always overrides IconTheme in Material3', (
      WidgetTester tester,
    ) async {
      // When IconButtonTheme is closer to IconButton, IconButtonTheme overrides IconTheme
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData.from(colorScheme: const ColorScheme.light()),
          home: IconTheme.merge(
            data: const IconThemeData(color: Colors.orange, size: 36),
            child: IconButtonTheme(
              data: IconButtonThemeData(
                style: IconButton.styleFrom(foregroundColor: Colors.blue, iconSize: 35),
              ),
              child: IconButton(icon: const Icon(Icons.account_box), onPressed: () {}),
            ),
          ),
        ),
      );

      Color? iconColor0() => _iconStyle(tester, Icons.account_box)?.color;
      expect(iconColor0(), Colors.blue);
      expect(tester.getSize(find.byIcon(Icons.account_box)), equals(const Size(35, 35)));

      // When IconTheme is closer to IconButton, IconButtonTheme still overrides IconTheme
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData.from(colorScheme: const ColorScheme.light()),
          home: IconTheme.merge(
            data: const IconThemeData(color: Colors.blue, size: 35),
            child: IconButtonTheme(
              data: IconButtonThemeData(
                style: IconButton.styleFrom(foregroundColor: Colors.orange, iconSize: 36),
              ),
              child: IconButton(icon: const Icon(Icons.account_box), onPressed: () {}),
            ),
          ),
        ),
      );

      Color? iconColor1() => _iconStyle(tester, Icons.account_box)?.color;
      expect(iconColor1(), Colors.orange);
      expect(tester.getSize(find.byIcon(Icons.account_box)), equals(const Size(36, 36)));
    });

    testWidgets('White icon color defined by users shows correctly in Material3', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData.from(
            colorScheme: const ColorScheme.dark(),
          ).copyWith(iconTheme: const IconThemeData(color: Colors.white)),
          home: IconButton(icon: const Icon(Icons.account_box), onPressed: () {}),
        ),
      );

      Color? iconColor1() => _iconStyle(tester, Icons.account_box)?.color;
      expect(iconColor1(), Colors.white);
    });

    testWidgets(
      'In light mode, icon color is M3 default color instead of IconTheme.of(context).color, '
      'if only setting color in IconTheme',
      (WidgetTester tester) async {
        final ColorScheme darkScheme = const ColorScheme.dark().copyWith(
          onSurfaceVariant: const Color(0xffe91e60),
        );
        // Brightness.dark
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(colorScheme: darkScheme),
            home: Scaffold(
              body: IconTheme.merge(
                data: const IconThemeData(size: 26),
                child: IconButton(icon: const Icon(Icons.account_box), onPressed: () {}),
              ),
            ),
          ),
        );

        Color? iconColor0() => _iconStyle(tester, Icons.account_box)?.color;
        expect(iconColor0(), darkScheme.onSurfaceVariant); // onSurfaceVariant
      },
    );

    testWidgets(
      'In dark mode, icon color is M3 default color instead of IconTheme.of(context).color, '
      'if only setting color in IconTheme',
      (WidgetTester tester) async {
        final ColorScheme lightScheme = const ColorScheme.light().copyWith(
          onSurfaceVariant: const Color(0xffe91e60),
        );
        // Brightness.dark
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(colorScheme: lightScheme),
            home: Scaffold(
              body: IconTheme.merge(
                data: const IconThemeData(size: 26),
                child: IconButton(icon: const Icon(Icons.account_box), onPressed: () {}),
              ),
            ),
          ),
        );

        Color? iconColor0() => _iconStyle(tester, Icons.account_box)?.color;
        expect(iconColor0(), lightScheme.onSurfaceVariant); // onSurfaceVariant
      },
    );

    testWidgets(
      'black87 icon color defined by users shows correctly in Material3',
      (WidgetTester tester) async {},
    );

    testWidgets("IconButton.styleFrom doesn't throw exception on passing only one cursor", (
      WidgetTester tester,
    ) async {
      // This is a regression test for https://github.com/flutter/flutter/issues/118071.
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: IconButton(
              style: OutlinedButton.styleFrom(enabledMouseCursor: SystemMouseCursors.text),
              onPressed: () {},
              icon: const Icon(Icons.add),
            ),
          ),
        ),
      );

      expect(tester.takeException(), isNull);
    });

    testWidgets('Material3 - IconButton memory leak', (WidgetTester tester) async {
      // This is a regression test for https://github.com/flutter/flutter/issues/130708.
      Widget buildWidget(bool showIconButton) {
        return showIconButton
            ? MaterialApp(home: IconButton(onPressed: () {}, icon: const Icon(Icons.search)))
            : const SizedBox();
      }

      await tester.pumpWidget(buildWidget(true));
      await tester.pumpWidget(buildWidget(false));

      // No exception is thrown.
    });
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/153544.
  testWidgets('Tooltip is drawn when hovering within IconButton area but outside the Icon itself', (
    WidgetTester tester,
  ) async {
    const String tooltipText = 'Test tooltip';
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: IconButton(
              onPressed: () {},
              icon: const Icon(Icons.favorite),
              tooltip: tooltipText,
            ),
          ),
        ),
      ),
    );

    // Verify that the tooltip is not shown initially.
    expect(findTooltipContainer(tooltipText), findsNothing);

    // Start hovering within IconButton area to show the tooltip.
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();

    final Offset topLeft = tester.getTopLeft(find.byType(Icon));
    // Move the cursor just outside the Icon.
    await gesture.moveTo(Offset(topLeft.dx - 1, topLeft.dy - 1));
    await tester.pump();

    // Verify that the tooltip is shown.
    expect(findTooltipContainer(tooltipText), findsOneWidget);
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/153544.
  testWidgets('Trigger Ink splash when hovering within layout bounds with tooltip', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: ColoredBox(
              color: const Color(0xFFFF0000),
              child: IconButton(
                onPressed: () {},
                icon: const Icon(Icons.favorite),
                tooltip: 'Test tooltip',
                style: const ButtonStyle(
                  overlayColor: WidgetStatePropertyAll<Color>(Color(0xFF00FF00)),
                  padding: WidgetStatePropertyAll<EdgeInsets>(EdgeInsets.all(20)),
                ),
              ),
            ),
          ),
        ),
      ),
    );

    final Offset topLeft = tester.getTopLeft(find.byType(ColoredBox));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(topLeft);
    await gesture.down(topLeft);
    await tester.pumpAndSettle();
    expect(
      getOverlayColor(tester),
      paints
        ..rect(color: const Color(0xFFFF0000)) // ColoredBox.
        ..rect(color: const Color(0xFF00FF00)), // IconButton overlay.
    );
  });

  testWidgets('Material3 - IconButton variants hovered & onLongPressed', (
    WidgetTester tester,
  ) async {
    late bool onHovered;
    bool onLongPressed = false;

    void onLongPress() {
      onLongPressed = true;
    }

    void onHover(bool hover) {
      onHovered = hover;
    }

    // IconButton
    await tester.pumpWidget(buildAllVariants(onLongPress: onLongPress, onHover: onHover));

    final Finder iconButton = find.widgetWithIcon(IconButton, Icons.favorite);
    final Offset iconButtonOffset = tester.getCenter(iconButton);
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    addTearDown(gesture.removePointer);

    await gesture.moveTo(iconButtonOffset);
    await tester.pump();
    expect(onHovered, true);

    await tester.longPressAt(iconButtonOffset);
    await tester.pump();
    expect(onLongPressed, true);

    onHovered = false;
    onLongPressed = false;

    await tester.pumpWidget(
      buildAllVariants(enabled: false, onLongPress: onLongPress, onHover: onHover),
    );
    await gesture.moveTo(iconButtonOffset);
    await tester.pump();
    expect(onHovered, false);

    await tester.longPressAt(iconButtonOffset);
    await tester.pump();
    expect(onLongPressed, false);

    await gesture.removePointer();

    // IconButton.filled
    await tester.pumpWidget(buildAllVariants(onLongPress: onLongPress, onHover: onHover));

    final Finder iconButtonFilled = find.widgetWithIcon(IconButton, Icons.add);
    final Offset iconButtonFilledOffset = tester.getCenter(iconButtonFilled);

    await gesture.moveTo(iconButtonFilledOffset);
    await tester.pump();
    expect(onHovered, true);

    await tester.longPressAt(iconButtonFilledOffset);
    await tester.pump();
    expect(onLongPressed, true);

    onHovered = false;
    onLongPressed = false;

    await tester.pumpWidget(
      buildAllVariants(enabled: false, onLongPress: onLongPress, onHover: onHover),
    );
    await gesture.moveTo(iconButtonFilledOffset);
    await tester.pump();
    expect(onHovered, false);

    await tester.longPressAt(iconButtonFilledOffset);
    await tester.pump();
    expect(onLongPressed, false);

    await gesture.removePointer();

    // IconButton.filledTonal
    await tester.pumpWidget(buildAllVariants(onLongPress: onLongPress, onHover: onHover));

    final Finder iconButtonFilledTonal = find.widgetWithIcon(IconButton, Icons.add);
    final Offset iconButtonFilledTonalOffset = tester.getCenter(iconButtonFilledTonal);

    await gesture.moveTo(iconButtonFilledTonalOffset);
    await tester.pump();
    expect(onHovered, true);

    await tester.longPressAt(iconButtonFilledTonalOffset);
    await tester.pump();
    expect(onLongPressed, true);

    onHovered = false;
    onLongPressed = false;

    await tester.pumpWidget(
      buildAllVariants(enabled: false, onLongPress: onLongPress, onHover: onHover),
    );
    await gesture.moveTo(iconButtonFilledTonalOffset);
    await tester.pump();
    expect(onHovered, false);

    await tester.longPressAt(iconButtonFilledTonalOffset);
    await tester.pump();
    expect(onLongPressed, false);

    await gesture.removePointer();

    // IconButton.outlined
    await tester.pumpWidget(buildAllVariants(onLongPress: onLongPress, onHover: onHover));

    final Finder iconButtonOutlined = find.widgetWithIcon(IconButton, Icons.add);
    final Offset iconButtonOutlinedOffset = tester.getCenter(iconButtonOutlined);

    await gesture.moveTo(iconButtonOutlinedOffset);
    await tester.pump();
    expect(onHovered, true);

    await tester.longPressAt(iconButtonOutlinedOffset);
    await tester.pump();
    expect(onLongPressed, true);

    onHovered = false;
    onLongPressed = false;

    await tester.pumpWidget(
      buildAllVariants(enabled: false, onLongPress: onLongPress, onHover: onHover),
    );
    await gesture.moveTo(iconButtonOutlinedOffset);
    await tester.pump();
    expect(onHovered, false);

    await tester.longPressAt(iconButtonOutlinedOffset);
    await tester.pump();
    expect(onLongPressed, false);
  });

  testWidgets('Material2 - IconButton variants hovered & onLongPressed', (
    WidgetTester tester,
  ) async {
    late bool onHovered;
    bool onLongPressed = false;

    void onLongPress() {
      onLongPressed = true;
    }

    void onHover(bool hover) {
      onHovered = hover;
    }

    // IconButton
    await tester.pumpWidget(
      buildAllVariants(onLongPress: onLongPress, onHover: onHover, useMaterial3: false),
    );

    final Finder iconButton = find.widgetWithIcon(IconButton, Icons.favorite);
    final Offset iconButtonOffset = tester.getCenter(iconButton);
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    addTearDown(gesture.removePointer);

    await gesture.moveTo(iconButtonOffset);
    await tester.pump();
    expect(onHovered, true);

    await tester.longPressAt(iconButtonOffset);
    await tester.pump();
    expect(onLongPressed, true);

    onHovered = false;
    onLongPressed = false;

    await tester.pumpWidget(
      buildAllVariants(
        enabled: false,
        onLongPress: onLongPress,
        onHover: onHover,
        useMaterial3: false,
      ),
    );
    await gesture.moveTo(iconButtonOffset);
    await tester.pump();
    expect(onHovered, false);

    await tester.longPressAt(iconButtonOffset);
    await tester.pump();
    expect(onLongPressed, false);

    await gesture.removePointer();

    // IconButton.filled
    await tester.pumpWidget(
      buildAllVariants(onLongPress: onLongPress, onHover: onHover, useMaterial3: false),
    );

    final Finder iconButtonFilled = find.widgetWithIcon(IconButton, Icons.add);
    final Offset iconButtonFilledOffset = tester.getCenter(iconButtonFilled);

    await gesture.moveTo(iconButtonFilledOffset);
    await tester.pump();
    expect(onHovered, true);

    await tester.longPressAt(iconButtonFilledOffset);
    await tester.pump();
    expect(onLongPressed, true);

    onHovered = false;
    onLongPressed = false;

    await tester.pumpWidget(
      buildAllVariants(
        enabled: false,
        onLongPress: onLongPress,
        onHover: onHover,
        useMaterial3: false,
      ),
    );
    await gesture.moveTo(iconButtonFilledOffset);
    await tester.pump();
    expect(onHovered, false);

    await tester.longPressAt(iconButtonFilledOffset);
    await tester.pump();
    expect(onLongPressed, false);

    await gesture.removePointer();

    // IconButton.filledTonal
    await tester.pumpWidget(
      buildAllVariants(onLongPress: onLongPress, onHover: onHover, useMaterial3: false),
    );

    final Finder iconButtonFilledTonal = find.widgetWithIcon(IconButton, Icons.add);
    final Offset iconButtonFilledTonalOffset = tester.getCenter(iconButtonFilledTonal);

    await gesture.moveTo(iconButtonFilledTonalOffset);
    await tester.pump();
    expect(onHovered, true);

    await tester.longPressAt(iconButtonFilledTonalOffset);
    await tester.pump();
    expect(onLongPressed, true);

    onHovered = false;
    onLongPressed = false;

    await tester.pumpWidget(
      buildAllVariants(
        enabled: false,
        onLongPress: onLongPress,
        onHover: onHover,
        useMaterial3: false,
      ),
    );
    await gesture.moveTo(iconButtonFilledTonalOffset);
    await tester.pump();
    expect(onHovered, false);

    await tester.longPressAt(iconButtonFilledTonalOffset);
    await tester.pump();
    expect(onLongPressed, false);

    await gesture.removePointer();

    // IconButton.outlined
    await tester.pumpWidget(
      buildAllVariants(onLongPress: onLongPress, onHover: onHover, useMaterial3: false),
    );

    final Finder iconButtonOutlined = find.widgetWithIcon(IconButton, Icons.add);
    final Offset iconButtonOutlinedOffset = tester.getCenter(iconButtonOutlined);

    await gesture.moveTo(iconButtonOutlinedOffset);
    await tester.pump();
    expect(onHovered, true);

    await tester.longPressAt(iconButtonOutlinedOffset);
    await tester.pump();
    expect(onLongPressed, true);

    onHovered = false;
    onLongPressed = false;

    await tester.pumpWidget(
      buildAllVariants(
        enabled: false,
        onLongPress: onLongPress,
        onHover: onHover,
        useMaterial3: false,
      ),
    );
    await gesture.moveTo(iconButtonOutlinedOffset);
    await tester.pump();
    expect(onHovered, false);

    await tester.longPressAt(iconButtonOutlinedOffset);
    await tester.pump();
    expect(onLongPressed, false);
  });

  testWidgets('does not draw focus color when focused by semantics on the web', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/158527.

    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    const Color focusColor = Colors.orange;

    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: IconButton(
            focusColor: focusColor,
            focusNode: focusNode,
            icon: const Icon(Icons.headphones),
            onPressed: () {},
          ),
        ),
      ),
    );

    // Make sure we are in "traditional mode" where the button could potentially draw focus highlights.
    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pumpAndSettle();
    expect(FocusManager.instance.highlightMode, equals(FocusHighlightMode.traditional));

    expect(focusNode.hasFocus, isFalse);

    // Focus on it with semantics.
    tester.platformDispatcher.onSemanticsActionEvent!(
      SemanticsActionEvent(
        type: SemanticsAction.focus,
        viewId: tester.view.viewId,
        nodeId: tester.semantics.find(find.byIcon(Icons.headphones)).id,
      ),
    );
    await tester.pumpAndSettle();

    // Make sure no focus highlight was drawn.
    final RenderObject inkFeatures = tester.allRenderObjects.firstWhere((RenderObject object) {
      return object.runtimeType.toString() == '_RenderInkFeatures';
    });
    expect(focusNode.hasFocus, isTrue);
    expect(FocusManager.instance.highlightMode, equals(FocusHighlightMode.touch));
    expect(inkFeatures, isNot(paints..rect(color: focusColor)));

    // Check that focus highlight is drawn in traditional mode.
    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pumpAndSettle();
    expect(focusNode.hasFocus, isTrue);
    expect(FocusManager.instance.highlightMode, equals(FocusHighlightMode.traditional));
    expect(inkFeatures, paints..rect(color: focusColor));
  }, skip: !isBrowser); // [intended] tests web-specific behavior.
}

Widget buildAllVariants({
  bool enabled = true,
  bool useMaterial3 = true,
  void Function(bool)? onHover,
  VoidCallback? onLongPress,
}) {
  return MaterialApp(
    theme: ThemeData(useMaterial3: useMaterial3),
    home: Material(
      child: Directionality(
        textDirection: TextDirection.ltr,
        child: Column(
          children: <Widget>[
            IconButton(
              icon: const Icon(Icons.favorite),
              onPressed: enabled ? () {} : null,
              onHover: onHover,
              onLongPress: onLongPress,
            ),
            IconButton.filled(
              icon: const Icon(Icons.add),
              onPressed: enabled ? () {} : null,
              onHover: onHover,
              onLongPress: onLongPress,
            ),
            IconButton.filledTonal(
              icon: const Icon(Icons.settings),
              onPressed: enabled ? () {} : null,
              onHover: onHover,
              onLongPress: onLongPress,
            ),
            IconButton.outlined(
              icon: const Icon(Icons.home),
              onPressed: enabled ? () {} : null,
              onHover: onHover,
              onLongPress: onLongPress,
            ),
          ],
        ),
      ),
    ),
  );
}

Widget wrap({required Widget child, required bool useMaterial3}) {
  return useMaterial3
      ? MaterialApp(
        theme: ThemeData.from(colorScheme: const ColorScheme.light()),
        home: FocusTraversalGroup(
          policy: ReadingOrderTraversalPolicy(),
          child: Directionality(textDirection: TextDirection.ltr, child: Center(child: child)),
        ),
      )
      : FocusTraversalGroup(
        policy: ReadingOrderTraversalPolicy(),
        child: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(child: Center(child: child)),
        ),
      );
}

TextStyle? _iconStyle(WidgetTester tester, IconData icon) {
  final RichText iconRichText = tester.widget<RichText>(
    find.descendant(of: find.byIcon(icon), matching: find.byType(RichText)),
  );
  return iconRichText.text.style;
}
