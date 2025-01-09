// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

Widget wrap({required Widget child, ThemeData? theme}) {
  return MaterialApp(theme: theme, home: Center(child: Material(child: child)));
}

void main() {
  testWidgets('ExpandIcon test', (WidgetTester tester) async {
    bool expanded = false;
    IconTheme iconTheme;

    // Light mode tests
    await tester.pumpWidget(
      wrap(
        child: ExpandIcon(
          onPressed: (bool isExpanded) {
            expanded = !expanded;
          },
        ),
      ),
    );
    await tester.pumpAndSettle();

    expect(expanded, isFalse);
    iconTheme = tester.firstWidget(find.byType(IconTheme).last);
    expect(iconTheme.data.color, equals(Colors.black54));

    await tester.tap(find.byType(ExpandIcon));
    await tester.pumpAndSettle();
    expect(expanded, isTrue);
    iconTheme = tester.firstWidget(find.byType(IconTheme).last);
    expect(iconTheme.data.color, equals(Colors.black54));

    await tester.tap(find.byType(ExpandIcon));
    await tester.pumpAndSettle();
    expect(expanded, isFalse);
    iconTheme = tester.firstWidget(find.byType(IconTheme).last);
    expect(iconTheme.data.color, equals(Colors.black54));

    // Dark mode tests
    await tester.pumpWidget(
      wrap(
        child: ExpandIcon(
          onPressed: (bool isExpanded) {
            expanded = !expanded;
          },
        ),
        theme: ThemeData(brightness: Brightness.dark),
      ),
    );
    await tester.pumpAndSettle();

    expect(expanded, isFalse);
    iconTheme = tester.firstWidget(find.byType(IconTheme).last);
    expect(iconTheme.data.color, equals(Colors.white60));

    await tester.tap(find.byType(ExpandIcon));
    await tester.pumpAndSettle();
    expect(expanded, isTrue);
    iconTheme = tester.firstWidget(find.byType(IconTheme).last);
    expect(iconTheme.data.color, equals(Colors.white60));

    await tester.tap(find.byType(ExpandIcon));
    await tester.pumpAndSettle();
    expect(expanded, isFalse);
    iconTheme = tester.firstWidget(find.byType(IconTheme).last);
    expect(iconTheme.data.color, equals(Colors.white60));
  });

  testWidgets('Material2 - ExpandIcon disabled', (WidgetTester tester) async {
    IconTheme iconTheme;
    // Test light mode.
    await tester.pumpWidget(
      wrap(theme: ThemeData(useMaterial3: false), child: const ExpandIcon(onPressed: null)),
    );
    await tester.pumpAndSettle();

    iconTheme = tester.firstWidget(find.byType(IconTheme).last);
    expect(iconTheme.data.color, equals(Colors.black38));

    // Test dark mode.
    await tester.pumpWidget(
      wrap(
        child: const ExpandIcon(onPressed: null),
        theme: ThemeData(useMaterial3: false, brightness: Brightness.dark),
      ),
    );
    await tester.pumpAndSettle();

    iconTheme = tester.firstWidget(find.byType(IconTheme).last);
    expect(iconTheme.data.color, equals(Colors.white38));
  });

  testWidgets('Material3 - ExpandIcon disabled', (WidgetTester tester) async {
    ThemeData theme = ThemeData();
    IconTheme iconTheme;
    // Test light mode.
    await tester.pumpWidget(wrap(theme: theme, child: const ExpandIcon(onPressed: null)));
    await tester.pumpAndSettle();

    iconTheme = tester.firstWidget(find.byType(IconTheme).last);
    expect(iconTheme.data.color, equals(theme.colorScheme.onSurface.withOpacity(0.38)));

    theme = ThemeData(brightness: Brightness.dark);
    // Test dark mode.
    await tester.pumpWidget(wrap(theme: theme, child: const ExpandIcon(onPressed: null)));
    await tester.pumpAndSettle();

    iconTheme = tester.firstWidget(find.byType(IconTheme).last);
    expect(iconTheme.data.color, equals(theme.colorScheme.onSurface.withOpacity(0.38)));
  });

  testWidgets('ExpandIcon test isExpanded does not trigger callback', (WidgetTester tester) async {
    bool expanded = false;

    await tester.pumpWidget(
      wrap(
        child: ExpandIcon(
          onPressed: (bool isExpanded) {
            expanded = !expanded;
          },
        ),
      ),
    );

    await tester.pumpWidget(
      wrap(
        child: ExpandIcon(
          isExpanded: true,
          onPressed: (bool isExpanded) {
            expanded = !expanded;
          },
        ),
      ),
    );

    expect(expanded, isFalse);
  });

  testWidgets('ExpandIcon is rotated initially if isExpanded is true on first build', (
    WidgetTester tester,
  ) async {
    bool expanded = true;

    await tester.pumpWidget(
      wrap(
        child: ExpandIcon(
          isExpanded: expanded,
          onPressed: (bool isExpanded) {
            expanded = !isExpanded;
          },
        ),
      ),
    );
    final RotationTransition rotation = tester.firstWidget(find.byType(RotationTransition));
    expect(rotation.turns.value, 0.5);
  });

  testWidgets('ExpandIcon default size is 24', (WidgetTester tester) async {
    final ExpandIcon expandIcon = ExpandIcon(onPressed: (bool isExpanded) {});

    await tester.pumpWidget(wrap(child: expandIcon));

    final ExpandIcon icon = tester.firstWidget(find.byWidget(expandIcon));
    expect(icon.size, 24);
  });

  testWidgets('ExpandIcon has the correct given size', (WidgetTester tester) async {
    ExpandIcon expandIcon = ExpandIcon(size: 36, onPressed: (bool isExpanded) {});

    await tester.pumpWidget(wrap(child: expandIcon));

    ExpandIcon icon = tester.firstWidget(find.byWidget(expandIcon));
    expect(icon.size, 36);

    expandIcon = ExpandIcon(size: 48, onPressed: (bool isExpanded) {});

    await tester.pumpWidget(wrap(child: expandIcon));

    icon = tester.firstWidget(find.byWidget(expandIcon));
    expect(icon.size, 48);
  });

  testWidgets('Material2 - ExpandIcon has correct semantic hints', (WidgetTester tester) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    const DefaultMaterialLocalizations localizations = DefaultMaterialLocalizations();
    await tester.pumpWidget(
      wrap(
        theme: ThemeData(useMaterial3: false),
        child: ExpandIcon(isExpanded: true, onPressed: (bool _) {}),
      ),
    );

    expect(
      tester.getSemantics(find.byType(ExpandIcon)),
      matchesSemantics(
        hasTapAction: true,
        hasFocusAction: true,
        hasEnabledState: true,
        isEnabled: true,
        isFocusable: true,
        isButton: true,
        onTapHint: localizations.expandedIconTapHint,
      ),
    );

    await tester.pumpWidget(
      wrap(theme: ThemeData(useMaterial3: false), child: ExpandIcon(onPressed: (bool _) {})),
    );

    expect(
      tester.getSemantics(find.byType(ExpandIcon)),
      matchesSemantics(
        hasTapAction: true,
        hasFocusAction: true,
        hasEnabledState: true,
        isEnabled: true,
        isFocusable: true,
        isButton: true,
        onTapHint: localizations.collapsedIconTapHint,
      ),
    );
    handle.dispose();
  });

  testWidgets('Material3 - ExpandIcon has correct semantic hints', (WidgetTester tester) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    const DefaultMaterialLocalizations localizations = DefaultMaterialLocalizations();

    await tester.pumpWidget(wrap(child: ExpandIcon(isExpanded: true, onPressed: (bool _) {})));

    expect(
      tester.getSemantics(find.byType(ExpandIcon)),
      matchesSemantics(
        onTapHint: localizations.expandedIconTapHint,
        children: <Matcher>[
          matchesSemantics(
            hasTapAction: true,
            hasFocusAction: true,
            hasEnabledState: true,
            isEnabled: true,
            isFocusable: true,
            isButton: true,
          ),
        ],
      ),
    );

    await tester.pumpWidget(wrap(child: ExpandIcon(onPressed: (bool _) {})));

    expect(
      tester.getSemantics(find.byType(ExpandIcon)),
      matchesSemantics(
        onTapHint: localizations.collapsedIconTapHint,
        children: <Matcher>[
          matchesSemantics(
            hasTapAction: true,
            hasFocusAction: true,
            hasEnabledState: true,
            isEnabled: true,
            isFocusable: true,
            isButton: true,
          ),
        ],
      ),
    );

    handle.dispose();
  });

  testWidgets('ExpandIcon uses custom icon color and expanded icon color', (
    WidgetTester tester,
  ) async {
    bool expanded = false;
    IconTheme iconTheme;

    await tester.pumpWidget(
      wrap(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return ExpandIcon(
              isExpanded: expanded,
              onPressed: (bool isExpanded) {
                setState(() {
                  expanded = !isExpanded;
                });
              },
              color: Colors.indigo,
            );
          },
        ),
      ),
    );
    await tester.pumpAndSettle();
    iconTheme = tester.firstWidget(find.byType(IconTheme).last);
    expect(iconTheme.data.color, equals(Colors.indigo));

    await tester.tap(find.byType(ExpandIcon));
    await tester.pumpAndSettle();
    iconTheme = tester.firstWidget(find.byType(IconTheme).last);
    expect(iconTheme.data.color, equals(Colors.indigo));

    expanded = false;

    await tester.pumpWidget(
      wrap(
        child: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return ExpandIcon(
              isExpanded: expanded,
              onPressed: (bool isExpanded) {
                setState(() {
                  expanded = !isExpanded;
                });
              },
              color: Colors.indigo,
              expandedColor: Colors.teal,
            );
          },
        ),
      ),
    );
    await tester.pumpAndSettle();
    iconTheme = tester.firstWidget(find.byType(IconTheme).last);
    expect(iconTheme.data.color, equals(Colors.indigo));

    await tester.tap(find.byType(ExpandIcon));
    await tester.pumpAndSettle();
    iconTheme = tester.firstWidget(find.byType(IconTheme).last);
    expect(iconTheme.data.color, equals(Colors.teal));

    await tester.tap(find.byType(ExpandIcon));
    await tester.pumpAndSettle();
    iconTheme = tester.firstWidget(find.byType(IconTheme).last);
    expect(iconTheme.data.color, equals(Colors.indigo));
  });

  testWidgets('ExpandIcon uses custom disabled icon color', (WidgetTester tester) async {
    IconTheme iconTheme;

    await tester.pumpWidget(
      wrap(child: const ExpandIcon(onPressed: null, disabledColor: Colors.cyan)),
    );
    await tester.pumpAndSettle();
    iconTheme = tester.firstWidget(find.byType(IconTheme).last);
    expect(iconTheme.data.color, equals(Colors.cyan));

    await tester.pumpWidget(
      wrap(
        child: const ExpandIcon(onPressed: null, color: Colors.indigo, disabledColor: Colors.cyan),
      ),
    );
    await tester.pumpAndSettle();
    iconTheme = tester.firstWidget(find.byType(IconTheme).last);
    expect(iconTheme.data.color, equals(Colors.cyan));

    await tester.pumpWidget(
      wrap(child: const ExpandIcon(isExpanded: true, onPressed: null, disabledColor: Colors.cyan)),
    );
    await tester.pumpWidget(
      wrap(
        child: const ExpandIcon(
          isExpanded: true,
          onPressed: null,
          expandedColor: Colors.teal,
          disabledColor: Colors.cyan,
        ),
      ),
    );
    await tester.pumpAndSettle();
    iconTheme = tester.firstWidget(find.byType(IconTheme).last);
    expect(iconTheme.data.color, equals(Colors.cyan));
  });
}
