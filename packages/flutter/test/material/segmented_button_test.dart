// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/semantics_tester.dart';

void main() {
  Widget boilerplate({required Widget child}) {
    return Directionality(textDirection: TextDirection.ltr, child: Center(child: child));
  }

  TextStyle iconStyle(WidgetTester tester, IconData icon) {
    final RichText iconRichText = tester.widget<RichText>(
      find.descendant(of: find.byIcon(icon), matching: find.byType(RichText)),
    );
    return iconRichText.text.style!;
  }

  testWidgets('SegmentsButton when compositing does not crash', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/135747
    // If the render object holds on to a stale canvas reference, this will
    // throw an exception.
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: SegmentedButton<int>(
            segments: const <ButtonSegment<int>>[
              ButtonSegment<int>(
                value: 0,
                label: Opacity(opacity: 0.5, child: Text('option')),
                icon: Opacity(opacity: 0.5, child: Icon(Icons.add)),
              ),
            ],
            selected: const <int>{0},
          ),
        ),
      ),
    );

    expect(find.byType(SegmentedButton<int>), findsOneWidget);
    expect(tester.takeException(), isNull);
  });

  testWidgets('SegmentedButton releases state controllers for deleted segments', (
    WidgetTester tester,
  ) async {
    final ThemeData theme = ThemeData(useMaterial3: true);
    final Key key = UniqueKey();

    Widget buildApp(Widget button) {
      return MaterialApp(theme: theme, home: Scaffold(body: Center(child: button)));
    }

    await tester.pumpWidget(
      buildApp(
        SegmentedButton<int>(
          key: key,
          segments: const <ButtonSegment<int>>[
            ButtonSegment<int>(value: 1, label: Text('1')),
            ButtonSegment<int>(value: 2, label: Text('2')),
          ],
          selected: const <int>{2},
        ),
      ),
    );

    await tester.pumpWidget(
      buildApp(
        SegmentedButton<int>(
          key: key,
          segments: const <ButtonSegment<int>>[
            ButtonSegment<int>(value: 2, label: Text('2')),
            ButtonSegment<int>(value: 3, label: Text('3')),
          ],
          selected: const <int>{2},
        ),
      ),
    );

    final SegmentedButtonState<int> state = tester.state(find.byType(SegmentedButton<int>));
    expect(state.statesControllers, hasLength(2));
    expect(state.statesControllers.keys.first.value, 2);
    expect(state.statesControllers.keys.last.value, 3);
  });

  testWidgets('SegmentedButton is built with Material of type MaterialType.transparency', (
    WidgetTester tester,
  ) async {
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

    // Expect SegmentedButton to be built with type MaterialType.transparency.
    final Finder text = find.text('1');
    final Finder parent = find.ancestor(of: text, matching: find.byType(Material)).first;
    final Finder parentMaterial = find.ancestor(of: parent, matching: find.byType(Material)).first;
    final Material material = tester.widget<Material>(parentMaterial);
    expect(material.type, MaterialType.transparency);
  });

  testWidgets('SegmentedButton supports exclusive choice by default', (WidgetTester tester) async {
    int callbackCount = 0;
    int selectedSegment = 2;

    Widget frameWithSelection(int selected) {
      return Material(
        child: boilerplate(
          child: SegmentedButton<int>(
            segments: const <ButtonSegment<int>>[
              ButtonSegment<int>(value: 1, label: Text('1')),
              ButtonSegment<int>(value: 2, label: Text('2')),
              ButtonSegment<int>(value: 3, label: Text('3')),
            ],
            selected: <int>{selected},
            onSelectionChanged: (Set<int> selected) {
              assert(selected.length == 1);
              selectedSegment = selected.first;
              callbackCount += 1;
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(frameWithSelection(selectedSegment));
    expect(selectedSegment, 2);
    expect(callbackCount, 0);

    // Tap on segment 1.
    await tester.tap(find.text('1'));
    await tester.pumpAndSettle();
    expect(callbackCount, 1);
    expect(selectedSegment, 1);

    // Update the selection in the widget
    await tester.pumpWidget(frameWithSelection(1));

    // Tap on segment 1 again should do nothing.
    await tester.tap(find.text('1'));
    await tester.pumpAndSettle();
    expect(callbackCount, 1);
    expect(selectedSegment, 1);

    // Tap on segment 3.
    await tester.tap(find.text('3'));
    await tester.pumpAndSettle();
    expect(callbackCount, 2);
    expect(selectedSegment, 3);
  });

  testWidgets('SegmentedButton supports multiple selected segments', (WidgetTester tester) async {
    int callbackCount = 0;
    Set<int> selection = <int>{1};

    Widget frameWithSelection(Set<int> selected) {
      return Material(
        child: boilerplate(
          child: SegmentedButton<int>(
            multiSelectionEnabled: true,
            segments: const <ButtonSegment<int>>[
              ButtonSegment<int>(value: 1, label: Text('1')),
              ButtonSegment<int>(value: 2, label: Text('2')),
              ButtonSegment<int>(value: 3, label: Text('3')),
            ],
            selected: selected,
            onSelectionChanged: (Set<int> selected) {
              selection = selected;
              callbackCount += 1;
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(frameWithSelection(selection));
    expect(selection, <int>{1});
    expect(callbackCount, 0);

    // Tap on segment 2.
    await tester.tap(find.text('2'));
    await tester.pumpAndSettle();
    expect(callbackCount, 1);
    expect(selection, <int>{1, 2});

    // Update the selection in the widget
    await tester.pumpWidget(frameWithSelection(<int>{1, 2}));
    await tester.pumpAndSettle();

    // Tap on segment 1 again should remove it from selection.
    await tester.tap(find.text('1'));
    await tester.pumpAndSettle();
    expect(callbackCount, 2);
    expect(selection, <int>{2});

    // Update the selection in the widget
    await tester.pumpWidget(frameWithSelection(<int>{2}));
    await tester.pumpAndSettle();

    // Tap on segment 3.
    await tester.tap(find.text('3'));
    await tester.pumpAndSettle();
    expect(callbackCount, 3);
    expect(selection, <int>{2, 3});
  });

  testWidgets('SegmentedButton allows for empty selection', (WidgetTester tester) async {
    int callbackCount = 0;
    int? selectedSegment = 1;

    Widget frameWithSelection(int? selected) {
      return Material(
        child: boilerplate(
          child: SegmentedButton<int>(
            emptySelectionAllowed: true,
            segments: const <ButtonSegment<int>>[
              ButtonSegment<int>(value: 1, label: Text('1')),
              ButtonSegment<int>(value: 2, label: Text('2')),
              ButtonSegment<int>(value: 3, label: Text('3')),
            ],
            selected: <int>{if (selected != null) selected},
            onSelectionChanged: (Set<int> selected) {
              selectedSegment = selected.isEmpty ? null : selected.first;
              callbackCount += 1;
            },
          ),
        ),
      );
    }

    await tester.pumpWidget(frameWithSelection(selectedSegment));
    expect(selectedSegment, 1);
    expect(callbackCount, 0);

    // Tap on segment 1 should deselect it and make the selection empty.
    await tester.tap(find.text('1'));
    await tester.pumpAndSettle();
    expect(callbackCount, 1);
    expect(selectedSegment, null);

    // Update the selection in the widget
    await tester.pumpWidget(frameWithSelection(null));

    // Tap on segment 2 should select it.
    await tester.tap(find.text('2'));
    await tester.pumpAndSettle();
    expect(callbackCount, 2);
    expect(selectedSegment, 2);

    // Update the selection in the widget
    await tester.pumpWidget(frameWithSelection(2));

    // Tap on segment 3.
    await tester.tap(find.text('3'));
    await tester.pumpAndSettle();
    expect(callbackCount, 3);
    expect(selectedSegment, 3);
  });

  testWidgets('SegmentedButton shows checkboxes for selected segments', (
    WidgetTester tester,
  ) async {
    Widget frameWithSelection(int selected) {
      return Material(
        child: boilerplate(
          child: SegmentedButton<int>(
            segments: const <ButtonSegment<int>>[
              ButtonSegment<int>(value: 1, label: Text('1')),
              ButtonSegment<int>(value: 2, label: Text('2')),
              ButtonSegment<int>(value: 3, label: Text('3')),
            ],
            selected: <int>{selected},
            onSelectionChanged: (Set<int> selected) {},
          ),
        ),
      );
    }

    Finder textHasIcon(String text, IconData icon) {
      return find.descendant(of: find.widgetWithText(Row, text), matching: find.byIcon(icon));
    }

    await tester.pumpWidget(frameWithSelection(1));
    expect(textHasIcon('1', Icons.check), findsOneWidget);
    expect(find.byIcon(Icons.check), findsOneWidget);

    await tester.pumpWidget(frameWithSelection(2));
    expect(textHasIcon('2', Icons.check), findsOneWidget);
    expect(find.byIcon(Icons.check), findsOneWidget);

    await tester.pumpWidget(frameWithSelection(2));
    expect(textHasIcon('2', Icons.check), findsOneWidget);
    expect(find.byIcon(Icons.check), findsOneWidget);
  });

  testWidgets(
    'SegmentedButton shows selected checkboxes in place of icon if it has a label as well',
    (WidgetTester tester) async {
      Widget frameWithSelection(int selected) {
        return Material(
          child: boilerplate(
            child: SegmentedButton<int>(
              segments: const <ButtonSegment<int>>[
                ButtonSegment<int>(value: 1, icon: Icon(Icons.add), label: Text('1')),
                ButtonSegment<int>(value: 2, icon: Icon(Icons.add_a_photo), label: Text('2')),
                ButtonSegment<int>(value: 3, icon: Icon(Icons.add_alarm), label: Text('3')),
              ],
              selected: <int>{selected},
              onSelectionChanged: (Set<int> selected) {},
            ),
          ),
        );
      }

      Finder textHasIcon(String text, IconData icon) {
        return find.descendant(of: find.widgetWithText(Row, text), matching: find.byIcon(icon));
      }

      await tester.pumpWidget(frameWithSelection(1));
      expect(textHasIcon('1', Icons.check), findsOneWidget);
      expect(find.byIcon(Icons.add), findsNothing);
      expect(textHasIcon('2', Icons.add_a_photo), findsOneWidget);
      expect(textHasIcon('3', Icons.add_alarm), findsOneWidget);

      await tester.pumpWidget(frameWithSelection(2));
      expect(textHasIcon('1', Icons.add), findsOneWidget);
      expect(textHasIcon('2', Icons.check), findsOneWidget);
      expect(find.byIcon(Icons.add_a_photo), findsNothing);
      expect(textHasIcon('3', Icons.add_alarm), findsOneWidget);

      await tester.pumpWidget(frameWithSelection(3));
      expect(textHasIcon('1', Icons.add), findsOneWidget);
      expect(textHasIcon('2', Icons.add_a_photo), findsOneWidget);
      expect(textHasIcon('3', Icons.check), findsOneWidget);
      expect(find.byIcon(Icons.add_alarm), findsNothing);
    },
  );

  testWidgets('SegmentedButton shows selected checkboxes next to icon if there is no label', (
    WidgetTester tester,
  ) async {
    Widget frameWithSelection(int selected) {
      return Material(
        child: boilerplate(
          child: SegmentedButton<int>(
            segments: const <ButtonSegment<int>>[
              ButtonSegment<int>(value: 1, icon: Icon(Icons.add)),
              ButtonSegment<int>(value: 2, icon: Icon(Icons.add_a_photo)),
              ButtonSegment<int>(value: 3, icon: Icon(Icons.add_alarm)),
            ],
            selected: <int>{selected},
            onSelectionChanged: (Set<int> selected) {},
          ),
        ),
      );
    }

    Finder rowWithIcons(IconData icon1, IconData icon2) {
      return find.descendant(of: find.widgetWithIcon(Row, icon1), matching: find.byIcon(icon2));
    }

    await tester.pumpWidget(frameWithSelection(1));
    expect(rowWithIcons(Icons.add, Icons.check), findsOneWidget);
    expect(rowWithIcons(Icons.add_a_photo, Icons.check), findsNothing);
    expect(rowWithIcons(Icons.add_alarm, Icons.check), findsNothing);

    await tester.pumpWidget(frameWithSelection(2));
    expect(rowWithIcons(Icons.add, Icons.check), findsNothing);
    expect(rowWithIcons(Icons.add_a_photo, Icons.check), findsOneWidget);
    expect(rowWithIcons(Icons.add_alarm, Icons.check), findsNothing);

    await tester.pumpWidget(frameWithSelection(3));
    expect(rowWithIcons(Icons.add, Icons.check), findsNothing);
    expect(rowWithIcons(Icons.add_a_photo, Icons.check), findsNothing);
    expect(rowWithIcons(Icons.add_alarm, Icons.check), findsOneWidget);
  });

  testWidgets('SegmentedButtons have correct semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      Material(
        child: boilerplate(
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
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            // First is an unselected, enabled button.
            TestSemantics(
              flags: <SemanticsFlag>[
                SemanticsFlag.isButton,
                SemanticsFlag.isEnabled,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.hasCheckedState,
                SemanticsFlag.isFocusable,
                SemanticsFlag.isInMutuallyExclusiveGroup,
              ],
              label: '1',
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
            ),

            // Second is a selected, enabled button.
            TestSemantics(
              flags: <SemanticsFlag>[
                SemanticsFlag.isButton,
                SemanticsFlag.isEnabled,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.hasCheckedState,
                SemanticsFlag.isChecked,
                SemanticsFlag.isFocusable,
                SemanticsFlag.isInMutuallyExclusiveGroup,
              ],
              label: '2',
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
            ),

            // Third is an unselected, disabled button.
            TestSemantics(
              flags: <SemanticsFlag>[
                SemanticsFlag.isButton,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.hasCheckedState,
                SemanticsFlag.isInMutuallyExclusiveGroup,
              ],
              label: '3',
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

  testWidgets('Multi-select SegmentedButtons have correct semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    await tester.pumpWidget(
      Material(
        child: boilerplate(
          child: SegmentedButton<int>(
            segments: const <ButtonSegment<int>>[
              ButtonSegment<int>(value: 1, label: Text('1')),
              ButtonSegment<int>(value: 2, label: Text('2')),
              ButtonSegment<int>(value: 3, label: Text('3'), enabled: false),
            ],
            selected: const <int>{1, 3},
            onSelectionChanged: (Set<int> selected) {},
            multiSelectionEnabled: true,
          ),
        ),
      ),
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            // First is selected, enabled button.
            TestSemantics(
              flags: <SemanticsFlag>[
                SemanticsFlag.isButton,
                SemanticsFlag.isEnabled,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.hasCheckedState,
                SemanticsFlag.isChecked,
                SemanticsFlag.isFocusable,
              ],
              label: '1',
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
            ),

            // Second is an unselected, enabled button.
            TestSemantics(
              flags: <SemanticsFlag>[
                SemanticsFlag.isButton,
                SemanticsFlag.isEnabled,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.hasCheckedState,
                SemanticsFlag.isFocusable,
              ],
              label: '2',
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
            ),

            // Third is a selected, disabled button.
            TestSemantics(
              flags: <SemanticsFlag>[
                SemanticsFlag.isButton,
                SemanticsFlag.hasEnabledState,
                SemanticsFlag.isChecked,
                SemanticsFlag.hasCheckedState,
              ],
              label: '3',
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

  testWidgets('SegmentedButton default overlayColor and foregroundColor resolve pressed state', (
    WidgetTester tester,
  ) async {
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
              ],
              selected: const <int>{1},
              onSelectionChanged: (Set<int> selected) {},
            ),
          ),
        ),
      ),
    );

    final Material material = tester.widget<Material>(
      find.descendant(of: find.byType(TextButton), matching: find.byType(Material)),
    );
    final BuildContext context = tester.element(find.text('2'));
    final SplashController controller = Material.of(context);

    // Hovered.
    final Offset center = tester.getCenter(find.text('2'));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(controller, paints..rect(color: theme.colorScheme.onSurface.withOpacity(0.08)));
    expect(material.textStyle?.color, theme.colorScheme.onSurface);

    // Highlighted (pressed).
    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(
      controller,
      paints
        ..rect()
        ..rect(color: theme.colorScheme.onSurface.withOpacity(0.1)),
    );
    expect(material.textStyle?.color, theme.colorScheme.onSurface);
  });

  testWidgets('SegmentedButton has no tooltips by default', (WidgetTester tester) async {
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

    expect(find.byType(Tooltip), findsNothing);
  });

  testWidgets('SegmentedButton has correct tooltips', (WidgetTester tester) async {
    final ThemeData theme = ThemeData(useMaterial3: true);
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Center(
            child: SegmentedButton<int>(
              segments: const <ButtonSegment<int>>[
                ButtonSegment<int>(value: 1, label: Text('1')),
                ButtonSegment<int>(value: 2, label: Text('2'), tooltip: 't2'),
                ButtonSegment<int>(value: 3, label: Text('3'), tooltip: 't3', enabled: false),
              ],
              selected: const <int>{2},
              onSelectionChanged: (Set<int> selected) {},
            ),
          ),
        ),
      ),
    );

    expect(find.byType(Tooltip), findsNWidgets(2));
    expect(find.byTooltip('t2'), findsOneWidget);
    expect(find.byTooltip('t3'), findsOneWidget);
  });

  testWidgets('SegmentedButton.styleFrom is applied to the SegmentedButton', (
    WidgetTester tester,
  ) async {
    const Color foregroundColor = Color(0xfffffff0);
    const Color backgroundColor = Color(0xfffffff1);
    const Color selectedBackgroundColor = Color(0xfffffff2);
    const Color selectedForegroundColor = Color(0xfffffff3);
    const Color disabledBackgroundColor = Color(0xfffffff4);
    const Color disabledForegroundColor = Color(0xfffffff5);
    const MouseCursor enabledMouseCursor = SystemMouseCursors.text;
    const MouseCursor disabledMouseCursor = SystemMouseCursors.grab;

    final ButtonStyle styleFromStyle = SegmentedButton.styleFrom(
      foregroundColor: foregroundColor,
      backgroundColor: backgroundColor,
      selectedForegroundColor: selectedForegroundColor,
      selectedBackgroundColor: selectedBackgroundColor,
      disabledForegroundColor: disabledForegroundColor,
      disabledBackgroundColor: disabledBackgroundColor,
      shadowColor: const Color(0xfffffff6),
      surfaceTintColor: const Color(0xfffffff7),
      elevation: 1,
      textStyle: const TextStyle(color: Color(0xfffffff8)),
      padding: const EdgeInsets.all(2),
      side: const BorderSide(color: Color(0xfffffff9)),
      shape: const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(3))),
      enabledMouseCursor: enabledMouseCursor,
      disabledMouseCursor: disabledMouseCursor,
      visualDensity: VisualDensity.compact,
      tapTargetSize: MaterialTapTargetSize.shrinkWrap,
      animationDuration: const Duration(milliseconds: 100),
      enableFeedback: true,
      alignment: Alignment.center,
      splashFactory: NoSplash.splashFactory,
    );

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: SegmentedButton<int>(
              style: styleFromStyle,
              segments: const <ButtonSegment<int>>[
                ButtonSegment<int>(value: 1, label: Text('1')),
                ButtonSegment<int>(value: 2, label: Text('2')),
                ButtonSegment<int>(value: 3, label: Text('3'), enabled: false),
              ],
              selected: const <int>{2},
              onSelectionChanged: (Set<int> selected) {},
              selectedIcon: const Icon(Icons.alarm),
            ),
          ),
        ),
      ),
    );

    // Test provided button style is applied to the enabled button segment.
    ButtonStyle? buttonStyle = tester.widget<TextButton>(find.byType(TextButton).first).style;
    expect(buttonStyle?.foregroundColor?.resolve(enabled), foregroundColor);
    expect(buttonStyle?.backgroundColor?.resolve(enabled), backgroundColor);
    expect(buttonStyle?.overlayColor, styleFromStyle.overlayColor);
    expect(buttonStyle?.surfaceTintColor, styleFromStyle.surfaceTintColor);
    expect(buttonStyle?.elevation, styleFromStyle.elevation);
    expect(buttonStyle?.textStyle, styleFromStyle.textStyle);
    expect(buttonStyle?.padding, styleFromStyle.padding);
    expect(buttonStyle?.mouseCursor?.resolve(enabled), enabledMouseCursor);
    expect(buttonStyle?.visualDensity, styleFromStyle.visualDensity);
    expect(buttonStyle?.tapTargetSize, styleFromStyle.tapTargetSize);
    expect(buttonStyle?.animationDuration, styleFromStyle.animationDuration);
    expect(buttonStyle?.enableFeedback, styleFromStyle.enableFeedback);
    expect(buttonStyle?.alignment, styleFromStyle.alignment);
    expect(buttonStyle?.splashFactory, styleFromStyle.splashFactory);

    // Test provided button style is applied selected button segment.
    buttonStyle = tester.widget<TextButton>(find.byType(TextButton).at(1)).style;
    expect(buttonStyle?.foregroundColor?.resolve(selected), selectedForegroundColor);
    expect(buttonStyle?.backgroundColor?.resolve(selected), selectedBackgroundColor);
    expect(buttonStyle?.mouseCursor?.resolve(enabled), enabledMouseCursor);

    // Test provided button style is applied disabled button segment.
    buttonStyle = tester.widget<TextButton>(find.byType(TextButton).last).style;
    expect(buttonStyle?.foregroundColor?.resolve(disabled), disabledForegroundColor);
    expect(buttonStyle?.backgroundColor?.resolve(disabled), disabledBackgroundColor);
    expect(buttonStyle?.mouseCursor?.resolve(disabled), disabledMouseCursor);

    // Test provided button style is applied to the segmented button material.
    final Material material = tester.widget<Material>(
      find.descendant(of: find.byType(SegmentedButton<int>), matching: find.byType(Material)).first,
    );
    expect(material.elevation, styleFromStyle.elevation?.resolve(enabled));
    expect(material.shadowColor, styleFromStyle.shadowColor?.resolve(enabled));
    expect(material.surfaceTintColor, styleFromStyle.surfaceTintColor?.resolve(enabled));

    // Test provided button style border is applied to the segmented button border.
    expect(
      find.byType(SegmentedButton<int>),
      paints..line(color: styleFromStyle.side?.resolve(enabled)?.color),
    );

    // Test foreground color is applied to the overlay color.
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.down(tester.getCenter(find.text('1')));
    await tester.pumpAndSettle();
    final BuildContext context = tester.element(find.text('1'));
    expect(Material.of(context), paints..rect(color: foregroundColor.withOpacity(0.08)));
  });

  testWidgets('Disabled SegmentedButton has correct states when rebuilding', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Column(
                  children: <Widget>[
                    SegmentedButton<int>(
                      segments: const <ButtonSegment<int>>[
                        ButtonSegment<int>(value: 0, label: Text('foo')),
                      ],
                      selected: const <int>{0},
                    ),
                    ElevatedButton(
                      onPressed: () => setState(() {}),
                      child: const Text('Trigger rebuild'),
                    ),
                  ],
                );
              },
            ),
          ),
        ),
      ),
    );
    final Set<MaterialState> states = <MaterialState>{
      MaterialState.selected,
      MaterialState.disabled,
    };
    // Check the initial states.
    SegmentedButtonState<int> state = tester.state(find.byType(SegmentedButton<int>));
    expect(state.statesControllers.values.first.value, states);
    // Trigger a rebuild.
    await tester.tap(find.byType(ElevatedButton));
    await tester.pumpAndSettle();
    // Check the states after the rebuild.
    state = tester.state(find.byType(SegmentedButton<int>));
    expect(state.statesControllers.values.first.value, states);
  });

  testWidgets('Min button hit target height is 48.0 and min (painted) button height is 40 '
      'by default with standard density and MaterialTapTargetSize.padded', (
    WidgetTester tester,
  ) async {
    final ThemeData theme = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Center(
            child: Column(
              children: <Widget>[
                SegmentedButton<int>(
                  segments: const <ButtonSegment<int>>[
                    ButtonSegment<int>(
                      value: 0,
                      label: Text('Day'),
                      icon: Icon(Icons.calendar_view_day),
                    ),
                    ButtonSegment<int>(
                      value: 1,
                      label: Text('Week'),
                      icon: Icon(Icons.calendar_view_week),
                    ),
                    ButtonSegment<int>(
                      value: 2,
                      label: Text('Month'),
                      icon: Icon(Icons.calendar_view_month),
                    ),
                    ButtonSegment<int>(
                      value: 3,
                      label: Text('Year'),
                      icon: Icon(Icons.calendar_today),
                    ),
                  ],
                  selected: const <int>{0},
                  onSelectionChanged: (Set<int> value) {},
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(theme.visualDensity, VisualDensity.standard);
    expect(theme.materialTapTargetSize, MaterialTapTargetSize.padded);

    final Finder button = find.byType(SegmentedButton<int>);
    expect(tester.getSize(button).height, 48.0);
    expect(
      find.byType(SegmentedButton<int>),
      paints..rrect(
        style: PaintingStyle.stroke,
        strokeWidth: 1.0,
        // Button border height is button.bottom(43.5) - button.top(4.5) + stoke width(1) = 40.
        rrect: RRect.fromLTRBR(0.5, 4.5, 497.5, 43.5, const Radius.circular(19.5)),
      ),
    );
  });

  testWidgets(
    'SegmentedButton expands to fill the available width when expandedInsets is not null',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: SegmentedButton<int>(
                segments: const <ButtonSegment<int>>[
                  ButtonSegment<int>(value: 1, label: Text('Segment 1')),
                  ButtonSegment<int>(value: 2, label: Text('Segment 2')),
                ],
                selected: const <int>{1},
                expandedInsets: EdgeInsets.zero,
              ),
            ),
          ),
        ),
      );

      // Get the width of the SegmentedButton.
      final RenderBox box = tester.renderObject(find.byType(SegmentedButton<int>));
      final double segmentedButtonWidth = box.size.width;

      // Get the width of the parent widget.
      final double screenWidth = tester.getSize(find.byType(Scaffold)).width;

      // The width of the SegmentedButton must be equal to the width of the parent widget.
      expect(segmentedButtonWidth, equals(screenWidth));
    },
  );

  testWidgets('SegmentedButton does not expand when expandedInsets is null', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: SegmentedButton<int>(
              segments: const <ButtonSegment<int>>[
                ButtonSegment<int>(value: 1, label: Text('Segment 1')),
                ButtonSegment<int>(value: 2, label: Text('Segment 2')),
              ],
              selected: const <int>{1},
            ),
          ),
        ),
      ),
    );

    // Get the width of the SegmentedButton.
    final RenderBox box = tester.renderObject(find.byType(SegmentedButton<int>));
    final double segmentedButtonWidth = box.size.width;

    // Get the width of the parent widget.
    final double screenWidth = tester.getSize(find.byType(Scaffold)).width;

    // The width of the SegmentedButton must be less than the width of the parent widget.
    expect(segmentedButtonWidth, lessThan(screenWidth));
  }, skip: kIsWeb && !isCanvasKit); // https://github.com/flutter/flutter/issues/145527

  testWidgets('SegmentedButton.styleFrom overlayColor overrides default overlay color', (
    WidgetTester tester,
  ) async {
    const Color overlayColor = Color(0xffff0000);
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: SegmentedButton<int>(
              style: IconButton.styleFrom(overlayColor: overlayColor),
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
  });

  testWidgets('SegmentedButton.styleFrom with transparent overlayColor', (
    WidgetTester tester,
  ) async {
    const Color overlayColor = Colors.transparent;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: SegmentedButton<int>(
              style: IconButton.styleFrom(overlayColor: overlayColor),
              segments: const <ButtonSegment<int>>[
                ButtonSegment<int>(value: 0, label: Text('Option')),
              ],
              onSelectionChanged: (Set<int> selected) {},
              selected: const <int>{0},
            ),
          ),
        ),
      ),
    );

    // Hovered,
    final Offset center = tester.getCenter(find.text('Option'));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    final BuildContext context = tester.element(find.text('Option'));
    final SplashController controller = Material.of(context);
    expect(controller, paints..rect(color: overlayColor));

    // Highlighted (pressed).
    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(
      controller,
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
    expect(controller, paints..rect(color: overlayColor));
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/144990.
  testWidgets('SegmentedButton clips border path when drawing segments', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: SegmentedButton<int>(
              segments: const <ButtonSegment<int>>[
                ButtonSegment<int>(value: 0, label: Text('Option 1')),
                ButtonSegment<int>(value: 1, label: Text('Option 2')),
              ],
              onSelectionChanged: (Set<int> selected) {},
              selected: const <int>{0},
            ),
          ),
        ),
      ),
    );

    expect(
      find.byType(SegmentedButton<int>),
      paints
        ..save()
        ..clipPath() // Clip the border.
        ..path(color: const Color(0xffe8def8)) // Draw segment 0.
        ..save()
        ..clipPath() // Clip the border.
        ..path(color: const Color(0x00000000)), // Draw segment 1.
    );
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/144990.
  testWidgets('SegmentedButton dividers matches border rect size', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: SegmentedButton<int>(
              segments: const <ButtonSegment<int>>[
                ButtonSegment<int>(value: 0, label: Text('Option 1')),
                ButtonSegment<int>(value: 1, label: Text('Option 2')),
              ],
              onSelectionChanged: (Set<int> selected) {},
              selected: const <int>{0},
            ),
          ),
        ),
      ),
    );

    const double tapTargetSize = 48.0;
    expect(
      find.byType(SegmentedButton<int>),
      paints..line(
        p1: const Offset(166.8000030517578, 4.0),
        p2: const Offset(166.8000030517578, tapTargetSize - 4.0),
      ),
    );
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/99933

  testWidgets('SegmentedButton vertical aligned children', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: SegmentedButton<int>(
              segments: const <ButtonSegment<int>>[
                ButtonSegment<int>(value: 0, label: Text('Option 0')),
                ButtonSegment<int>(value: 1, label: Text('Option 1')),
                ButtonSegment<int>(value: 2, label: Text('Option 2')),
                ButtonSegment<int>(value: 3, label: Text('Option 3')),
              ],
              onSelectionChanged: (Set<int> selected) {},
              selected: const <int>{-1}, // Prevent any of ButtonSegment to be selected
              direction: Axis.vertical,
            ),
          ),
        ),
      ),
    );

    Rect? previewsChildRect;
    for (int i = 0; i <= 3; i++) {
      final Rect currentChildRect = tester.getRect(find.widgetWithText(TextButton, 'Option $i'));
      if (previewsChildRect != null) {
        expect(currentChildRect.left, previewsChildRect.left);
        expect(currentChildRect.right, previewsChildRect.right);
        expect(currentChildRect.top, previewsChildRect.top + previewsChildRect.height);
      }
      previewsChildRect = currentChildRect;
    }
  });

  testWidgets('SegmentedButton vertical aligned golden image', (WidgetTester tester) async {
    final GlobalKey key = GlobalKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: RepaintBoundary(
              key: key,
              child: SegmentedButton<int>(
                segments: const <ButtonSegment<int>>[
                  ButtonSegment<int>(value: 0, label: Text('Option 0')),
                  ButtonSegment<int>(value: 1, label: Text('Option 1')),
                ],
                selected: const <int>{0}, // Prevent any of ButtonSegment to be selected
                direction: Axis.vertical,
              ),
            ),
          ),
        ),
      ),
    );

    await expectLater(find.byKey(key), matchesGoldenFile('segmented_button_test_vertical.png'));
  });

  // Regression test for https://github.com/flutter/flutter/issues/154798.
  testWidgets('SegmentedButton.styleFrom can customize the button icon', (
    WidgetTester tester,
  ) async {
    const Color iconColor = Color(0xFFF000FF);
    const double iconSize = 32.0;
    const Color disabledIconColor = Color(0xFFFFF000);
    Widget buildButton({bool enabled = true}) {
      return MaterialApp(
        home: Material(
          child: Center(
            child: SegmentedButton<int>(
              style: SegmentedButton.styleFrom(
                iconColor: iconColor,
                iconSize: iconSize,
                disabledIconColor: disabledIconColor,
              ),
              segments: const <ButtonSegment<int>>[
                ButtonSegment<int>(value: 0, label: Text('Add'), icon: Icon(Icons.add)),
                ButtonSegment<int>(value: 1, label: Text('Subtract'), icon: Icon(Icons.remove)),
              ],
              showSelectedIcon: false,
              onSelectionChanged: enabled ? (Set<int> selected) {} : null,
              selected: const <int>{0},
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
  });
}

Set<MaterialState> enabled = const <MaterialState>{};
Set<MaterialState> disabled = const <MaterialState>{MaterialState.disabled};
Set<MaterialState> selected = const <MaterialState>{MaterialState.selected};
