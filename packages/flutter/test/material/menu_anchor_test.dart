// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker/leak_tracker.dart';

import '../widgets/semantics_tester.dart';

void main() {
  late MenuController controller;
  String? focusedMenu;
  final List<TestMenu> selected = <TestMenu>[];
  final List<TestMenu> opened = <TestMenu>[];
  final List<TestMenu> closed = <TestMenu>[];
  final GlobalKey menuItemKey = GlobalKey();

  void onPressed(TestMenu item) {
    selected.add(item);
  }

  void onOpen(TestMenu item) {
    opened.add(item);
  }

  void onClose(TestMenu item) {
    closed.add(item);
  }

  void handleFocusChange() {
    focusedMenu = (primaryFocus?.debugLabel ?? primaryFocus).toString();
  }

  setUp(() {
    focusedMenu = null;
    selected.clear();
    opened.clear();
    closed.clear();
    controller = MenuController();
    focusedMenu = null;
  });

  Future<void> changeSurfaceSize(WidgetTester tester, Size size) async {
    await tester.binding.setSurfaceSize(size);
    addTearDown(() async {
      await tester.binding.setSurfaceSize(null);
    });
  }

  void listenForFocusChanges() {
    FocusManager.instance.addListener(handleFocusChange);
    addTearDown(() => FocusManager.instance.removeListener(handleFocusChange));
  }

  Finder findMenuPanels() {
    return find.byWidgetPredicate((Widget widget) => widget.runtimeType.toString() == '_MenuPanel');
  }

  List<RenderObject> ancestorRenderTheaters(RenderObject child) {
    final List<RenderObject> results = <RenderObject>[];
    RenderObject? node = child;
    while (node != null) {
      if (node.runtimeType.toString() == '_RenderTheater') {
        results.add(node);
      }
      final RenderObject? parent = node.parent;
      node = parent is RenderObject ? parent : null;
    }
    return results;
  }

  Finder findMenuBarItemLabels() {
    return find.byWidgetPredicate(
      (Widget widget) => widget.runtimeType.toString() == '_MenuItemLabel',
    );
  }

  // Finds the mnemonic associated with the menu item that has the given label.
  Finder findMnemonic(String label) {
    return find
        .descendant(
          of: find.ancestor(of: find.text(label), matching: findMenuBarItemLabels()),
          matching: find.byType(Text),
        )
        .last;
  }

  Widget buildTestApp({
    AlignmentGeometry? alignment,
    Offset alignmentOffset = Offset.zero,
    TextDirection textDirection = TextDirection.ltr,
    bool consumesOutsideTap = false,
    void Function(TestMenu)? onPressed,
    void Function(TestMenu)? onOpen,
    void Function(TestMenu)? onClose,
  }) {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);
    return MaterialApp(
      theme: ThemeData(useMaterial3: false),
      home: Material(
        child: Directionality(
          textDirection: textDirection,
          child: Column(
            children: <Widget>[
              GestureDetector(
                onTap: () {
                  onPressed?.call(TestMenu.outsideButton);
                },
                child: Text(TestMenu.outsideButton.label),
              ),
              MenuAnchor(
                childFocusNode: focusNode,
                controller: controller,
                alignmentOffset: alignmentOffset,
                consumeOutsideTap: consumesOutsideTap,
                style: MenuStyle(alignment: alignment),
                onOpen: () {
                  onOpen?.call(TestMenu.anchorButton);
                },
                onClose: () {
                  onClose?.call(TestMenu.anchorButton);
                },
                menuChildren: <Widget>[
                  MenuItemButton(
                    key: menuItemKey,
                    shortcut: const SingleActivator(LogicalKeyboardKey.keyB, control: true),
                    onPressed: () {
                      onPressed?.call(TestMenu.subMenu00);
                    },
                    child: Text(TestMenu.subMenu00.label),
                  ),
                  MenuItemButton(
                    leadingIcon: const Icon(Icons.send),
                    trailingIcon: const Icon(Icons.mail),
                    onPressed: () {
                      onPressed?.call(TestMenu.subMenu01);
                    },
                    child: Text(TestMenu.subMenu01.label),
                  ),
                ],
                builder: (BuildContext context, MenuController controller, Widget? child) {
                  return ElevatedButton(
                    focusNode: focusNode,
                    onPressed: () {
                      if (controller.isOpen) {
                        controller.close();
                      } else {
                        controller.open();
                      }
                      onPressed?.call(TestMenu.anchorButton);
                    },
                    child: child,
                  );
                },
                child: Text(TestMenu.anchorButton.label),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Future<TestGesture> hoverOver(WidgetTester tester, Finder finder) async {
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.moveTo(tester.getCenter(finder));
    await tester.pumpAndSettle();
    return gesture;
  }

  Material getMenuBarMaterial(WidgetTester tester) {
    return tester.widget<Material>(
      find.descendant(of: findMenuPanels(), matching: find.byType(Material)).first,
    );
  }

  RenderObject getOverlayColor(WidgetTester tester) {
    return tester.allRenderObjects.firstWhere(
      (RenderObject object) => object.runtimeType.toString() == '_RenderInkFeatures',
    );
  }

  TextStyle iconStyle(WidgetTester tester, IconData icon) {
    final RichText iconRichText = tester.widget<RichText>(
      find.descendant(of: find.byIcon(icon), matching: find.byType(RichText)),
    );
    return iconRichText.text.style!;
  }

  testWidgets('Menu responds to density changes', (WidgetTester tester) async {
    Widget buildMenu({VisualDensity? visualDensity = VisualDensity.standard}) {
      return MaterialApp(
        theme: ThemeData(visualDensity: visualDensity, useMaterial3: false),
        home: Material(
          child: Column(
            children: <Widget>[
              MenuBar(children: createTestMenus(onPressed: onPressed)),
              const Expanded(child: Placeholder()),
            ],
          ),
        ),
      );
    }

    await tester.pumpWidget(buildMenu());
    await tester.pump();

    expect(
      tester.getRect(find.byType(MenuBar)),
      equals(const Rect.fromLTRB(145.0, 0.0, 655.0, 48.0)),
    );

    // Open and make sure things are the right size.
    await tester.tap(find.text(TestMenu.mainMenu1.label));
    await tester.pump();

    expect(
      tester.getRect(find.byType(MenuBar)),
      equals(const Rect.fromLTRB(145.0, 0.0, 655.0, 48.0)),
    );
    expect(
      tester.getRect(find.byType(MenuBar)),
      equals(const Rect.fromLTRB(145.0, 0.0, 655.0, 48.0)),
    );
    expect(
      tester.getRect(find.widgetWithText(MenuItemButton, TestMenu.subMenu10.label)),
      equals(const Rect.fromLTRB(257.0, 56.0, 471.0, 104.0)),
    );
    expect(
      tester.getRect(
        find
            .ancestor(of: find.text(TestMenu.subMenu10.label), matching: find.byType(Material))
            .at(1),
      ),
      equals(const Rect.fromLTRB(257.0, 48.0, 471.0, 208.0)),
    );

    // Test compact visual density (-2, -2).
    await tester.pumpWidget(Container());
    await tester.pumpWidget(buildMenu(visualDensity: VisualDensity.compact));
    await tester.pump();

    // The original horizontal padding with standard visual density for menu buttons are 12 px, and the total length
    // for the menu bar is (655 - 145) = 510.
    // There are 4 buttons in the test menu bar, and with compact visual density,
    // the padding will reduce by abs(2 * (-2)) = 4. So the total length
    // now should reduce by abs(4 * 2 * (-4)) = 32, which would be 510 - 32 = 478, and
    // 478 = 639 - 161
    expect(
      tester.getRect(find.byType(MenuBar)),
      equals(const Rect.fromLTRB(161.0, 0.0, 639.0, 40.0)),
    );

    // Open and make sure things are the right size.
    await tester.tap(find.text(TestMenu.mainMenu1.label));
    await tester.pump();

    expect(
      tester.getRect(find.byType(MenuBar)),
      equals(const Rect.fromLTRB(161.0, 0.0, 639.0, 40.0)),
    );
    expect(
      tester.getRect(find.byType(MenuBar)),
      equals(const Rect.fromLTRB(161.0, 0.0, 639.0, 40.0)),
    );
    expect(
      tester.getRect(find.widgetWithText(MenuItemButton, TestMenu.subMenu10.label)),
      equals(const Rect.fromLTRB(265.0, 40.0, 467.0, 80.0)),
    );
    expect(
      tester.getRect(
        find
            .ancestor(of: find.text(TestMenu.subMenu10.label), matching: find.byType(Material))
            .at(1),
      ),
      equals(const Rect.fromLTRB(265.0, 40.0, 467.0, 160.0)),
    );

    await tester.pumpWidget(Container());
    await tester.pumpWidget(
      buildMenu(visualDensity: const VisualDensity(horizontal: 2.0, vertical: 2.0)),
    );
    await tester.pump();

    // Similarly, there are 4 buttons in the test menu bar, and with (2, 2) visual density,
    // the padding will increase by abs(2 * 4) = 8. So the total length for buttons
    // should increase by abs(4 * 2 * 8) = 64. The horizontal padding for the menu bar
    // increases by 2 * 8, so the total width increases to 510 + 64 + 16 = 590, and
    // 590 = 695 - 105
    expect(
      tester.getRect(find.byType(MenuBar)),
      equals(const Rect.fromLTRB(105.0, 0.0, 695.0, 72.0)),
    );

    // Open and make sure things are the right size.
    await tester.tap(find.text(TestMenu.mainMenu1.label));
    await tester.pump();

    expect(
      tester.getRect(find.byType(MenuBar)),
      equals(const Rect.fromLTRB(105.0, 0.0, 695.0, 72.0)),
    );
    expect(
      tester.getRect(find.byType(MenuBar)),
      equals(const Rect.fromLTRB(105.0, 0.0, 695.0, 72.0)),
    );
    expect(
      tester.getRect(find.widgetWithText(MenuItemButton, TestMenu.subMenu10.label)),
      equals(const Rect.fromLTRB(257.0, 80.0, 491.0, 136.0)),
    );
    expect(
      tester.getRect(
        find
            .ancestor(of: find.text(TestMenu.subMenu10.label), matching: find.byType(Material))
            .at(1),
      ),
      equals(const Rect.fromLTRB(249.0, 64.0, 499.0, 264.0)),
    );
  });

  testWidgets('Menu defaults', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Material(
          child: MenuBar(
            controller: controller,
            children: createTestMenus(onPressed: onPressed, onOpen: onOpen, onClose: onClose),
          ),
        ),
      ),
    );

    // Menu bar (horizontal menu).
    Finder menuMaterial =
        find.ancestor(of: find.byType(TextButton), matching: find.byType(Material)).first;

    Material material = tester.widget<Material>(menuMaterial);
    expect(opened, isEmpty);
    expect(material.color, themeData.colorScheme.surfaceContainer);
    expect(material.shadowColor, themeData.colorScheme.shadow);
    expect(material.surfaceTintColor, Colors.transparent);
    expect(material.elevation, 3.0);
    expect(
      material.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4.0))),
    );

    Finder buttonMaterial =
        find.descendant(of: find.byType(TextButton), matching: find.byType(Material)).first;
    material = tester.widget<Material>(buttonMaterial);
    expect(material.color, Colors.transparent);
    expect(material.elevation, 0.0);
    expect(material.shape, const RoundedRectangleBorder());
    expect(material.textStyle?.color, themeData.colorScheme.onSurface);
    expect(material.textStyle?.fontSize, 14.0);
    expect(material.textStyle?.height, 1.43);

    // Vertical menu.
    await tester.tap(find.text(TestMenu.mainMenu1.label));
    await tester.pump();

    menuMaterial =
        find
            .ancestor(
              of: find.widgetWithText(TextButton, TestMenu.subMenu10.label),
              matching: find.byType(Material),
            )
            .first;

    material = tester.widget<Material>(menuMaterial);
    expect(opened.last, equals(TestMenu.mainMenu1));
    expect(material.color, themeData.colorScheme.surfaceContainer);
    expect(material.shadowColor, themeData.colorScheme.shadow);
    expect(material.surfaceTintColor, Colors.transparent);
    expect(material.elevation, 3.0);
    expect(
      material.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4.0))),
    );

    buttonMaterial =
        find
            .descendant(
              of: find.widgetWithText(TextButton, TestMenu.subMenu10.label),
              matching: find.byType(Material),
            )
            .first;
    material = tester.widget<Material>(buttonMaterial);
    expect(material.color, Colors.transparent);
    expect(material.elevation, 0.0);
    expect(material.shape, const RoundedRectangleBorder());
    expect(material.textStyle?.color, themeData.colorScheme.onSurface);
    expect(material.textStyle?.fontSize, 14.0);
    expect(material.textStyle?.height, 1.43);

    await tester.tap(find.text(TestMenu.mainMenu0.label));
    await tester.pump();
    expect(find.byIcon(Icons.add), findsOneWidget);
    final RichText iconRichText = tester.widget<RichText>(
      find.descendant(of: find.byIcon(Icons.add), matching: find.byType(RichText)),
    );
    expect(iconRichText.text.style?.color, themeData.colorScheme.onSurfaceVariant);
  });

  testWidgets('Menu defaults - disabled', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Material(
          child: MenuBar(
            controller: controller,
            children: createTestMenus(onPressed: onPressed, onOpen: onOpen, onClose: onClose),
          ),
        ),
      ),
    );

    // Menu bar (horizontal menu).
    Finder menuMaterial =
        find
            .ancestor(
              of: find.widgetWithText(TextButton, TestMenu.mainMenu5.label),
              matching: find.byType(Material),
            )
            .first;

    Material material = tester.widget<Material>(menuMaterial);
    expect(opened, isEmpty);
    expect(material.color, themeData.colorScheme.surfaceContainer);
    expect(material.shadowColor, themeData.colorScheme.shadow);
    expect(material.surfaceTintColor, Colors.transparent);
    expect(material.elevation, 3.0);
    expect(
      material.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4.0))),
    );

    Finder buttonMaterial =
        find
            .descendant(
              of: find.widgetWithText(TextButton, TestMenu.mainMenu5.label),
              matching: find.byType(Material),
            )
            .first;
    material = tester.widget<Material>(buttonMaterial);
    expect(material.color, Colors.transparent);
    expect(material.elevation, 0.0);
    expect(material.shape, const RoundedRectangleBorder());
    expect(material.textStyle?.color, themeData.colorScheme.onSurface.withOpacity(0.38));

    // Vertical menu.
    await tester.tap(find.text(TestMenu.mainMenu2.label));
    await tester.pump();

    menuMaterial =
        find
            .ancestor(
              of: find.widgetWithText(TextButton, TestMenu.subMenu20.label),
              matching: find.byType(Material),
            )
            .first;

    material = tester.widget<Material>(menuMaterial);
    expect(material.color, themeData.colorScheme.surfaceContainer);
    expect(material.shadowColor, themeData.colorScheme.shadow);
    expect(material.surfaceTintColor, Colors.transparent);
    expect(material.elevation, 3.0);
    expect(
      material.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4.0))),
    );

    buttonMaterial =
        find
            .descendant(
              of: find.widgetWithText(TextButton, TestMenu.subMenu20.label),
              matching: find.byType(Material),
            )
            .first;
    material = tester.widget<Material>(buttonMaterial);
    expect(material.color, Colors.transparent);
    expect(material.elevation, 0.0);
    expect(material.shape, const RoundedRectangleBorder());
    expect(material.textStyle?.color, themeData.colorScheme.onSurface.withOpacity(0.38));

    expect(find.byIcon(Icons.ac_unit), findsOneWidget);
    final RichText iconRichText = tester.widget<RichText>(
      find.descendant(of: find.byIcon(Icons.ac_unit), matching: find.byType(RichText)),
    );
    expect(iconRichText.text.style?.color, themeData.colorScheme.onSurface.withOpacity(0.38));
  });

  testWidgets('Menu scrollbar inherits ScrollbarTheme', (WidgetTester tester) async {
    const ScrollbarThemeData scrollbarTheme = ScrollbarThemeData(
      thumbColor: MaterialStatePropertyAll<Color?>(Color(0xffff0000)),
      thumbVisibility: MaterialStatePropertyAll<bool?>(true),
    );
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(scrollbarTheme: scrollbarTheme),
        home: Material(
          child: MenuBar(
            children: <Widget>[
              SubmenuButton(
                menuChildren: <Widget>[
                  MenuItemButton(
                    style: ButtonStyle(
                      minimumSize: MaterialStateProperty.all<Size>(const Size.fromHeight(1000)),
                    ),
                    onPressed: () {},
                    child: const Text('Category'),
                  ),
                ],
                child: const Text('Main Menu'),
              ),
            ],
          ),
        ),
      ),
    );

    await tester.tap(find.text('Main Menu'));
    await tester.pumpAndSettle();

    // Test Scrollbar thumb color.
    expect(find.byType(Scrollbar).last, paints..rrect(color: const Color(0xffff0000)));

    // Close the menu.
    await tester.tapAt(const Offset(10.0, 10.0));
    await tester.pumpAndSettle();

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(scrollbarTheme: scrollbarTheme),
        home: Material(
          child: ScrollbarTheme(
            data: scrollbarTheme.copyWith(
              thumbColor: const MaterialStatePropertyAll<Color?>(Color(0xff00ff00)),
            ),
            child: MenuBar(
              children: <Widget>[
                SubmenuButton(
                  menuChildren: <Widget>[
                    MenuItemButton(
                      style: ButtonStyle(
                        minimumSize: MaterialStateProperty.all<Size>(const Size.fromHeight(1000)),
                      ),
                      onPressed: () {},
                      child: const Text('Category'),
                    ),
                  ],
                  child: const Text('Main Menu'),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('Main Menu'));
    await tester.pumpAndSettle();

    // Scrollbar thumb color should be updated.
    expect(find.byType(Scrollbar).last, paints..rrect(color: const Color(0xff00ff00)));
  }, variant: TargetPlatformVariant.desktop());

  testWidgets('Focus is returned to previous focus before invoking onPressed', (
    WidgetTester tester,
  ) async {
    final FocusNode buttonFocus = FocusNode(debugLabel: 'Button Focus');
    addTearDown(buttonFocus.dispose);
    FocusNode? focusInOnPressed;

    void onMenuSelected(TestMenu item) {
      focusInOnPressed = FocusManager.instance.primaryFocus;
    }

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Column(
            children: <Widget>[
              MenuBar(controller: controller, children: createTestMenus(onPressed: onMenuSelected)),
              ElevatedButton(
                autofocus: true,
                onPressed: () {},
                focusNode: buttonFocus,
                child: const Text('Press Me'),
              ),
            ],
          ),
        ),
      ),
    );

    await tester.pump();
    expect(FocusManager.instance.primaryFocus, equals(buttonFocus));

    await tester.tap(find.text(TestMenu.mainMenu1.label));
    await tester.pump();

    await tester.tap(find.text(TestMenu.subMenu11.label));
    await tester.pump();

    await tester.tap(find.text(TestMenu.subSubMenu110.label));
    await tester.pump();

    expect(focusInOnPressed, equals(buttonFocus));
    expect(FocusManager.instance.primaryFocus, equals(buttonFocus));
  });

  group('Menu functions', () {
    testWidgets('basic menu structure', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              controller: controller,
              children: createTestMenus(onPressed: onPressed, onOpen: onOpen, onClose: onClose),
            ),
          ),
        ),
      );

      expect(find.text(TestMenu.mainMenu0.label), findsOneWidget);
      expect(find.text(TestMenu.mainMenu1.label), findsOneWidget);
      expect(find.text(TestMenu.mainMenu2.label), findsOneWidget);
      expect(find.text(TestMenu.subMenu10.label), findsNothing);
      expect(find.text(TestMenu.subSubMenu110.label), findsNothing);
      expect(opened, isEmpty);

      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();

      expect(find.text(TestMenu.mainMenu0.label), findsOneWidget);
      expect(find.text(TestMenu.mainMenu1.label), findsOneWidget);
      expect(find.text(TestMenu.mainMenu2.label), findsOneWidget);
      expect(find.text(TestMenu.subMenu10.label), findsOneWidget);
      expect(find.text(TestMenu.subMenu11.label), findsOneWidget);
      expect(find.text(TestMenu.subMenu12.label), findsOneWidget);
      expect(find.text(TestMenu.subSubMenu110.label), findsNothing);
      expect(find.text(TestMenu.subSubMenu111.label), findsNothing);
      expect(find.text(TestMenu.subSubMenu112.label), findsNothing);
      expect(opened.last, equals(TestMenu.mainMenu1));
      opened.clear();

      await tester.tap(find.text(TestMenu.subMenu11.label));
      await tester.pump();

      expect(find.text(TestMenu.mainMenu0.label), findsOneWidget);
      expect(find.text(TestMenu.mainMenu1.label), findsOneWidget);
      expect(find.text(TestMenu.mainMenu2.label), findsOneWidget);
      expect(find.text(TestMenu.subMenu10.label), findsOneWidget);
      expect(find.text(TestMenu.subMenu11.label), findsOneWidget);
      expect(find.text(TestMenu.subMenu12.label), findsOneWidget);
      expect(find.text(TestMenu.subSubMenu110.label), findsOneWidget);
      expect(find.text(TestMenu.subSubMenu111.label), findsOneWidget);
      expect(find.text(TestMenu.subSubMenu112.label), findsOneWidget);
      expect(opened.last, equals(TestMenu.subMenu11));
    });

    testWidgets('geometry', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(
            child: Column(
              children: <Widget>[
                Row(
                  children: <Widget>[
                    Expanded(child: MenuBar(children: createTestMenus(onPressed: onPressed))),
                  ],
                ),
                const Expanded(child: Placeholder()),
              ],
            ),
          ),
        ),
      );
      await tester.pump();

      expect(tester.getRect(find.byType(MenuBar)), equals(const Rect.fromLTRB(0, 0, 800, 48)));

      // Open and make sure things are the right size.
      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();

      expect(tester.getRect(find.byType(MenuBar)), equals(const Rect.fromLTRB(0, 0, 800, 48)));
      expect(
        tester.getRect(find.text(TestMenu.subMenu10.label)),
        equals(const Rect.fromLTRB(124.0, 73.0, 314.0, 87.0)),
      );
      expect(
        tester.getRect(
          find
              .ancestor(of: find.text(TestMenu.subMenu10.label), matching: find.byType(Material))
              .at(1),
        ),
        equals(const Rect.fromLTRB(112.0, 48.0, 326.0, 208.0)),
      );

      // Test menu bar size when not expanded.
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Column(
              children: <Widget>[
                MenuBar(children: createTestMenus(onPressed: onPressed)),
                const Expanded(child: Placeholder()),
              ],
            ),
          ),
        ),
      );
      await tester.pump();

      expect(
        tester.getRect(find.byType(MenuBar)),
        equals(const Rect.fromLTRB(145.0, 0.0, 655.0, 48.0)),
      );
    });

    testWidgets('geometry with RTL direction', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(
            child: Directionality(
              textDirection: TextDirection.rtl,
              child: Column(
                children: <Widget>[
                  Row(
                    children: <Widget>[
                      Expanded(child: MenuBar(children: createTestMenus(onPressed: onPressed))),
                    ],
                  ),
                  const Expanded(child: Placeholder()),
                ],
              ),
            ),
          ),
        ),
      );
      await tester.pump();

      expect(tester.getRect(find.byType(MenuBar)), equals(const Rect.fromLTRB(0, 0, 800, 48)));

      // Open and make sure things are the right size.
      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();

      expect(tester.getRect(find.byType(MenuBar)), equals(const Rect.fromLTRB(0, 0, 800, 48)));
      expect(
        tester.getRect(find.text(TestMenu.subMenu10.label)),
        equals(const Rect.fromLTRB(486.0, 73.0, 676.0, 87.0)),
      );
      expect(
        tester.getRect(
          find
              .ancestor(of: find.text(TestMenu.subMenu10.label), matching: find.byType(Material))
              .at(1),
        ),
        equals(const Rect.fromLTRB(474.0, 48.0, 688.0, 208.0)),
      );

      // Close and make sure it goes back where it was.
      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();

      expect(tester.getRect(find.byType(MenuBar)), equals(const Rect.fromLTRB(0, 0, 800, 48)));

      // Test menu bar size when not expanded.
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Directionality(
              textDirection: TextDirection.rtl,
              child: Column(
                children: <Widget>[
                  MenuBar(children: createTestMenus(onPressed: onPressed)),
                  const Expanded(child: Placeholder()),
                ],
              ),
            ),
          ),
        ),
      );
      await tester.pump();

      expect(
        tester.getRect(find.byType(MenuBar)),
        equals(const Rect.fromLTRB(145.0, 0.0, 655.0, 48.0)),
      );
    });

    testWidgets('menu alignment and offset in LTR', (WidgetTester tester) async {
      await tester.pumpWidget(buildTestApp());

      final Rect buttonRect = tester.getRect(find.byType(ElevatedButton));
      expect(buttonRect, equals(const Rect.fromLTRB(328.0, 14.0, 472.0, 62.0)));

      final Finder findMenuScope =
          find.ancestor(of: find.byKey(menuItemKey), matching: find.byType(FocusScope)).first;

      // Open the menu and make sure things are the right size, in the right place.
      await tester.tap(find.text('Press Me'));
      await tester.pump();
      expect(tester.getRect(findMenuScope), equals(const Rect.fromLTRB(328.0, 62.0, 602.0, 174.0)));

      await tester.pumpWidget(buildTestApp(alignment: AlignmentDirectional.topStart));
      await tester.pump();
      expect(tester.getRect(findMenuScope), equals(const Rect.fromLTRB(328.0, 14.0, 602.0, 126.0)));

      await tester.pumpWidget(buildTestApp(alignment: AlignmentDirectional.center));
      await tester.pump();
      expect(tester.getRect(findMenuScope), equals(const Rect.fromLTRB(400.0, 38.0, 674.0, 150.0)));

      await tester.pumpWidget(buildTestApp(alignment: AlignmentDirectional.bottomEnd));
      await tester.pump();
      expect(tester.getRect(findMenuScope), equals(const Rect.fromLTRB(472.0, 62.0, 746.0, 174.0)));

      await tester.pumpWidget(buildTestApp(alignment: AlignmentDirectional.topStart));
      await tester.pump();

      final Rect menuRect = tester.getRect(findMenuScope);
      await tester.pumpWidget(
        buildTestApp(
          alignment: AlignmentDirectional.topStart,
          alignmentOffset: const Offset(10, 20),
        ),
      );
      await tester.pump();
      final Rect offsetMenuRect = tester.getRect(findMenuScope);
      expect(offsetMenuRect.topLeft - menuRect.topLeft, equals(const Offset(10, 20)));
    });

    testWidgets('menu alignment and offset in RTL', (WidgetTester tester) async {
      await tester.pumpWidget(buildTestApp(textDirection: TextDirection.rtl));

      final Rect buttonRect = tester.getRect(find.byType(ElevatedButton));
      expect(buttonRect, equals(const Rect.fromLTRB(328.0, 14.0, 472.0, 62.0)));

      final Finder findMenuScope =
          find
              .ancestor(of: find.text(TestMenu.subMenu00.label), matching: find.byType(FocusScope))
              .first;

      // Open the menu and make sure things are the right size, in the right place.
      await tester.tap(find.text('Press Me'));
      await tester.pump();
      expect(tester.getRect(findMenuScope), equals(const Rect.fromLTRB(198.0, 62.0, 472.0, 174.0)));

      await tester.pumpWidget(
        buildTestApp(textDirection: TextDirection.rtl, alignment: AlignmentDirectional.topStart),
      );
      await tester.pump();
      expect(tester.getRect(findMenuScope), equals(const Rect.fromLTRB(198.0, 14.0, 472.0, 126.0)));

      await tester.pumpWidget(
        buildTestApp(textDirection: TextDirection.rtl, alignment: AlignmentDirectional.center),
      );
      await tester.pump();
      expect(tester.getRect(findMenuScope), equals(const Rect.fromLTRB(126.0, 38.0, 400.0, 150.0)));

      await tester.pumpWidget(
        buildTestApp(textDirection: TextDirection.rtl, alignment: AlignmentDirectional.bottomEnd),
      );
      await tester.pump();
      expect(tester.getRect(findMenuScope), equals(const Rect.fromLTRB(54.0, 62.0, 328.0, 174.0)));

      await tester.pumpWidget(
        buildTestApp(textDirection: TextDirection.rtl, alignment: AlignmentDirectional.topStart),
      );
      await tester.pump();

      final Rect menuRect = tester.getRect(findMenuScope);
      await tester.pumpWidget(
        buildTestApp(
          textDirection: TextDirection.rtl,
          alignment: AlignmentDirectional.topStart,
          alignmentOffset: const Offset(10, 20),
        ),
      );
      await tester.pump();
      expect(
        tester.getRect(findMenuScope).topLeft - menuRect.topLeft,
        equals(const Offset(-10, 20)),
      );
    });

    testWidgets('menu position in LTR', (WidgetTester tester) async {
      await tester.pumpWidget(buildTestApp(alignmentOffset: const Offset(100, 50)));

      final Rect buttonRect = tester.getRect(find.byType(ElevatedButton));
      expect(buttonRect, equals(const Rect.fromLTRB(328.0, 14.0, 472.0, 62.0)));

      final Finder findMenuScope =
          find
              .ancestor(of: find.text(TestMenu.subMenu00.label), matching: find.byType(FocusScope))
              .first;

      // Open the menu and make sure things are the right size, in the right place.
      await tester.tap(find.text('Press Me'));
      await tester.pump();
      expect(
        tester.getRect(findMenuScope),
        equals(const Rect.fromLTRB(428.0, 112.0, 702.0, 224.0)),
      );

      // Now move the menu by calling open() again with a local position on the
      // anchor.
      controller.open(position: const Offset(200, 200));
      await tester.pump();
      expect(
        tester.getRect(findMenuScope),
        equals(const Rect.fromLTRB(526.0, 214.0, 800.0, 326.0)),
      );
    });

    testWidgets('menu position in RTL', (WidgetTester tester) async {
      await tester.pumpWidget(
        buildTestApp(alignmentOffset: const Offset(100, 50), textDirection: TextDirection.rtl),
      );

      final Rect buttonRect = tester.getRect(find.byType(ElevatedButton));
      expect(buttonRect, equals(const Rect.fromLTRB(328.0, 14.0, 472.0, 62.0)));
      expect(buttonRect, equals(const Rect.fromLTRB(328.0, 14.0, 472.0, 62.0)));

      final Finder findMenuScope =
          find
              .ancestor(of: find.text(TestMenu.subMenu00.label), matching: find.byType(FocusScope))
              .first;

      // Open the menu and make sure things are the right size, in the right place.
      await tester.tap(find.text('Press Me'));
      await tester.pump();
      expect(tester.getRect(findMenuScope), equals(const Rect.fromLTRB(98.0, 112.0, 372.0, 224.0)));

      // Now move the menu by calling open() again with a local position on the
      // anchor.
      controller.open(position: const Offset(400, 200));
      await tester.pump();
      expect(
        tester.getRect(findMenuScope),
        equals(const Rect.fromLTRB(526.0, 214.0, 800.0, 326.0)),
      );
    });

    testWidgets('works with Padding around menu and overlay', (WidgetTester tester) async {
      await tester.pumpWidget(
        Padding(
          padding: const EdgeInsets.all(10.0),
          child: MaterialApp(
            theme: ThemeData(useMaterial3: false),
            home: Material(
              child: Column(
                children: <Widget>[
                  Padding(
                    padding: const EdgeInsets.all(12.0),
                    child: Row(
                      children: <Widget>[
                        Expanded(child: MenuBar(children: createTestMenus(onPressed: onPressed))),
                      ],
                    ),
                  ),
                  const Expanded(child: Placeholder()),
                ],
              ),
            ),
          ),
        ),
      );
      await tester.pump();

      expect(
        tester.getRect(find.byType(MenuBar)),
        equals(const Rect.fromLTRB(22.0, 22.0, 778.0, 70.0)),
      );

      // Open and make sure things are the right size.
      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();

      expect(
        tester.getRect(find.byType(MenuBar)),
        equals(const Rect.fromLTRB(22.0, 22.0, 778.0, 70.0)),
      );
      expect(
        tester.getRect(find.text(TestMenu.subMenu10.label)),
        equals(const Rect.fromLTRB(146.0, 95.0, 336.0, 109.0)),
      );
      expect(
        tester.getRect(
          find
              .ancestor(of: find.text(TestMenu.subMenu10.label), matching: find.byType(Material))
              .at(1),
        ),
        equals(const Rect.fromLTRB(134.0, 70.0, 348.0, 230.0)),
      );

      // Close and make sure it goes back where it was.
      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();

      expect(
        tester.getRect(find.byType(MenuBar)),
        equals(const Rect.fromLTRB(22.0, 22.0, 778.0, 70.0)),
      );
    });

    testWidgets('works with Padding around menu and overlay with RTL direction', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        Padding(
          padding: const EdgeInsets.all(10.0),
          child: MaterialApp(
            theme: ThemeData(useMaterial3: false),
            home: Material(
              child: Directionality(
                textDirection: TextDirection.rtl,
                child: Column(
                  children: <Widget>[
                    Padding(
                      padding: const EdgeInsets.all(12.0),
                      child: Row(
                        children: <Widget>[
                          Expanded(child: MenuBar(children: createTestMenus(onPressed: onPressed))),
                        ],
                      ),
                    ),
                    const Expanded(child: Placeholder()),
                  ],
                ),
              ),
            ),
          ),
        ),
      );
      await tester.pump();

      expect(
        tester.getRect(find.byType(MenuBar)),
        equals(const Rect.fromLTRB(22.0, 22.0, 778.0, 70.0)),
      );

      // Open and make sure things are the right size.
      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();

      expect(
        tester.getRect(find.byType(MenuBar)),
        equals(const Rect.fromLTRB(22.0, 22.0, 778.0, 70.0)),
      );
      expect(
        tester.getRect(find.text(TestMenu.subMenu10.label)),
        equals(const Rect.fromLTRB(464.0, 95.0, 654.0, 109.0)),
      );
      expect(
        tester.getRect(
          find
              .ancestor(of: find.text(TestMenu.subMenu10.label), matching: find.byType(Material))
              .at(1),
        ),
        equals(const Rect.fromLTRB(452.0, 70.0, 666.0, 230.0)),
      );

      // Close and make sure it goes back where it was.
      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();

      expect(
        tester.getRect(find.byType(MenuBar)),
        equals(const Rect.fromLTRB(22.0, 22.0, 778.0, 70.0)),
      );
    });

    testWidgets('visual attributes can be set', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Column(
              children: <Widget>[
                Row(
                  children: <Widget>[
                    Expanded(
                      child: MenuBar(
                        style: MenuStyle(
                          elevation: MaterialStateProperty.all<double?>(10),
                          backgroundColor: const MaterialStatePropertyAll<Color>(Colors.red),
                        ),
                        children: createTestMenus(onPressed: onPressed),
                      ),
                    ),
                  ],
                ),
                const Expanded(child: Placeholder()),
              ],
            ),
          ),
        ),
      );
      expect(tester.getRect(findMenuPanels()), equals(const Rect.fromLTRB(0.0, 0.0, 800.0, 48.0)));
      final Material material = getMenuBarMaterial(tester);
      expect(material.elevation, equals(10));
      expect(material.color, equals(Colors.red));
    });

    testWidgets('MenuAnchor clip behavior', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: MenuAnchor(
                menuChildren: const <Widget>[MenuItemButton(child: Text('Button 1'))],
                builder: (BuildContext context, MenuController controller, Widget? child) {
                  return FilledButton(
                    onPressed: () {
                      controller.open();
                    },
                    child: const Text('Tap me'),
                  );
                },
              ),
            ),
          ),
        ),
      );
      await tester.tap(find.text('Tap me'));
      await tester.pump();

      // Test default clip behavior.
      expect(getMenuBarMaterial(tester).clipBehavior, equals(Clip.hardEdge));

      // Close the menu.
      await tester.tapAt(const Offset(10.0, 10.0));
      await tester.pumpAndSettle();
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: MenuAnchor(
                clipBehavior: Clip.antiAlias,
                menuChildren: const <Widget>[MenuItemButton(child: Text('Button 1'))],
                builder: (BuildContext context, MenuController controller, Widget? child) {
                  return FilledButton(
                    onPressed: () {
                      controller.open();
                    },
                    child: const Text('Tap me'),
                  );
                },
              ),
            ),
          ),
        ),
      );
      await tester.tap(find.text('Tap me'));
      await tester.pump();

      // Test custom clip behavior.
      expect(getMenuBarMaterial(tester).clipBehavior, equals(Clip.antiAlias));
    });

    testWidgets('open and close works', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              controller: controller,
              children: createTestMenus(onPressed: onPressed, onOpen: onOpen, onClose: onClose),
            ),
          ),
        ),
      );

      expect(opened, isEmpty);
      expect(closed, isEmpty);

      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();
      expect(opened, equals(<TestMenu>[TestMenu.mainMenu1]));
      expect(closed, isEmpty);
      opened.clear();
      closed.clear();

      await tester.tap(find.text(TestMenu.subMenu11.label));
      await tester.pump();

      expect(opened, equals(<TestMenu>[TestMenu.subMenu11]));
      expect(closed, isEmpty);
      opened.clear();
      closed.clear();

      await tester.tap(find.text(TestMenu.subMenu11.label));
      await tester.pump();

      expect(opened, isEmpty);
      expect(closed, equals(<TestMenu>[TestMenu.subMenu11]));
      opened.clear();
      closed.clear();

      await tester.tap(find.text(TestMenu.mainMenu0.label));
      await tester.pump();

      expect(opened, equals(<TestMenu>[TestMenu.mainMenu0]));
      expect(closed, equals(<TestMenu>[TestMenu.mainMenu1]));
    });

    testWidgets('Menus close and consume tap when open and tapped outside', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        buildTestApp(
          consumesOutsideTap: true,
          onPressed: onPressed,
          onOpen: onOpen,
          onClose: onClose,
        ),
      );

      expect(opened, isEmpty);
      expect(closed, isEmpty);

      // Doesn't consume tap when the menu is closed.
      await tester.tap(find.text(TestMenu.outsideButton.label));
      await tester.pump();
      expect(selected, equals(<TestMenu>[TestMenu.outsideButton]));
      selected.clear();

      await tester.tap(find.text(TestMenu.anchorButton.label));
      await tester.pump();
      expect(opened, equals(<TestMenu>[TestMenu.anchorButton]));
      expect(closed, isEmpty);
      expect(selected, equals(<TestMenu>[TestMenu.anchorButton]));
      opened.clear();
      closed.clear();
      selected.clear();

      await tester.tap(find.text(TestMenu.outsideButton.label));
      await tester.pump();

      expect(opened, isEmpty);
      expect(closed, equals(<TestMenu>[TestMenu.anchorButton]));
      // When the menu is open, don't expect the outside button to be selected:
      // it's supposed to consume the key down.
      expect(selected, isEmpty);
      selected.clear();
      opened.clear();
      closed.clear();
    });

    testWidgets("Menus close and don't consume tap when open and tapped outside", (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(buildTestApp(onPressed: onPressed, onOpen: onOpen, onClose: onClose));

      expect(opened, isEmpty);
      expect(closed, isEmpty);

      // Doesn't consume tap when the menu is closed.
      await tester.tap(find.text(TestMenu.outsideButton.label));
      await tester.pump();
      expect(selected, equals(<TestMenu>[TestMenu.outsideButton]));
      selected.clear();

      await tester.tap(find.text(TestMenu.anchorButton.label));
      await tester.pump();
      expect(opened, equals(<TestMenu>[TestMenu.anchorButton]));
      expect(closed, isEmpty);
      expect(selected, equals(<TestMenu>[TestMenu.anchorButton]));
      opened.clear();
      closed.clear();
      selected.clear();

      await tester.tap(find.text(TestMenu.outsideButton.label));
      await tester.pump();

      expect(opened, isEmpty);
      expect(closed, equals(<TestMenu>[TestMenu.anchorButton]));
      // Because consumesOutsideTap is false, this is expected to receive its
      // tap.
      expect(selected, equals(<TestMenu>[TestMenu.outsideButton]));
      selected.clear();
      opened.clear();
      closed.clear();
    });

    testWidgets('select works', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              controller: controller,
              children: createTestMenus(onPressed: onPressed, onOpen: onOpen, onClose: onClose),
            ),
          ),
        ),
      );

      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();

      await tester.tap(find.text(TestMenu.subMenu11.label));
      await tester.pump();

      expect(opened, equals(<TestMenu>[TestMenu.mainMenu1, TestMenu.subMenu11]));
      opened.clear();
      await tester.tap(find.text(TestMenu.subSubMenu110.label));
      await tester.pump();

      expect(selected, equals(<TestMenu>[TestMenu.subSubMenu110]));

      // Selecting a non-submenu item should close all the menus.
      expect(opened, isEmpty);
      expect(find.text(TestMenu.subSubMenu110.label), findsNothing);
      expect(find.text(TestMenu.subMenu11.label), findsNothing);
    });

    testWidgets('diagnostics', (WidgetTester tester) async {
      const MenuItemButton item = MenuItemButton(
        shortcut: SingleActivator(LogicalKeyboardKey.keyA),
        child: Text('label2'),
      );
      final MenuBar menuBar = MenuBar(
        controller: controller,
        style: const MenuStyle(
          backgroundColor: MaterialStatePropertyAll<Color>(Colors.red),
          elevation: MaterialStatePropertyAll<double?>(10.0),
        ),
        children: const <Widget>[item],
      );

      await tester.pumpWidget(MaterialApp(home: Material(child: menuBar)));
      await tester.pump();

      final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
      menuBar.debugFillProperties(builder);

      final List<String> description =
          builder.properties
              .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
              .map((DiagnosticsNode node) => node.toString())
              .toList();

      expect(
        description.join('\n'),
        equalsIgnoringHashCodes(
          'style: MenuStyle#00000(backgroundColor: WidgetStatePropertyAll(MaterialColor(primary value: ${const Color(0xfff44336)})), elevation: WidgetStatePropertyAll(10.0))\n'
          'clipBehavior: Clip.none',
        ),
      );
    });
    testWidgets('menus can be traversed multiple times', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/150334
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Column(
              children: <Widget>[
                MenuItemButton(
                  autofocus: true,
                  onPressed: () {},
                  child: const Text('External Focus'),
                ),
                MenuBar(
                  controller: controller,
                  children: createTestMenus(onPressed: onPressed, onOpen: onOpen, onClose: onClose),
                ),
              ],
            ),
          ),
        ),
      );

      listenForFocusChanges();

      // Have to open a menu initially to start things going.
      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 1"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 10"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('SubmenuButton(Text("Sub Menu 11"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.escape);
      await tester.pump();
      expect(focusedMenu, equals('MenuItemButton(Text("External Focus"))'));

      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 1"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 10"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('SubmenuButton(Text("Sub Menu 11"))'));
    });

    testWidgets('keyboard tab traversal works', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Column(
              children: <Widget>[
                MenuBar(
                  controller: controller,
                  children: createTestMenus(onPressed: onPressed, onOpen: onOpen, onClose: onClose),
                ),
                const Expanded(child: Placeholder()),
              ],
            ),
          ),
        ),
      );

      listenForFocusChanges();

      // Have to open a menu initially to start things going.
      await tester.tap(find.text(TestMenu.mainMenu0.label));
      await tester.pumpAndSettle();

      expect(focusedMenu, equals('SubmenuButton(Text("Menu 0"))'));
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 1"))'));
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 2"))'));
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 0"))'));

      await tester.sendKeyDownEvent(LogicalKeyboardKey.shiftLeft);
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 2"))'));
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 1"))'));
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 0"))'));
      await tester.sendKeyUpEvent(LogicalKeyboardKey.shiftLeft);
      opened.clear();
      closed.clear();

      // Test closing a menu with enter.
      await tester.sendKeyEvent(LogicalKeyboardKey.enter);
      await tester.pump();
      expect(opened, isEmpty);
      expect(closed, <TestMenu>[TestMenu.mainMenu0]);
    });

    testWidgets('keyboard directional traversal works', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              controller: controller,
              children: createTestMenus(onPressed: onPressed, onOpen: onOpen, onClose: onClose),
            ),
          ),
        ),
      );

      listenForFocusChanges();

      // Have to open a menu initially to start things going.
      await tester.tap(find.text(TestMenu.mainMenu0.label));
      await tester.pumpAndSettle();

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 1"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 10"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('SubmenuButton(Text("Sub Menu 11"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 12"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 12"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Sub Menu 11"))'));

      // Open the next submenu.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pump();
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Sub Menu 110"))'));

      // Go back, close the submenu.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Sub Menu 11"))'));

      // Move up, should close the submenu.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      await tester.pump();
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 10"))'));

      // Move down, should reopen the submenu.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Sub Menu 11"))'));

      // Open the next submenu again.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pump();
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Sub Menu 110"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Sub Menu 111"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Sub Menu 112"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Sub Menu 113"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Sub Menu 113"))'));

      // Since this is a leaf off of a vertical menu, moving left should
      // return to this menu's parent button.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      expect(focusedMenu, equals('SubmenuButton(Text("Sub Menu 11"))'));

      // Moving left while in a first-level submenu should focus the
      // previous top-level menubar anchor.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 0"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 1"))'));

      // Pressing arrowup from a top-level menubar anchor should focus the last
      // item in that anchor's submenu.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 12"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      expect(focusedMenu, equals('SubmenuButton(Text("Sub Menu 11"))'));
      await tester.pump();

      // Enter the submenu.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Sub Menu 110"))'));

      // Move to next top-level menu button.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 2"))'));
    });

    testWidgets('keyboard directional traversal works in RTL mode', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Directionality(
            textDirection: TextDirection.rtl,
            child: Material(
              child: MenuBar(
                controller: controller,
                children: createTestMenus(onPressed: onPressed, onOpen: onOpen, onClose: onClose),
              ),
            ),
          ),
        ),
      );

      listenForFocusChanges();

      // Have to open a menu initially to start things going.
      await tester.tap(find.text(TestMenu.mainMenu0.label));
      await tester.pump();

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 1"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 1"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 10"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('SubmenuButton(Text("Sub Menu 11"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 12"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 12"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Sub Menu 11"))'));

      // Open the next submenu.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pump();
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Sub Menu 110"))'));

      // Go back, close the submenu.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Sub Menu 11"))'));

      // Move up, should close the submenu.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      await tester.pump();
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 10"))'));

      // Move down, should reopen the submenu.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Sub Menu 11"))'));

      // Open the next submenu again.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pump();
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Sub Menu 110"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Sub Menu 111"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Sub Menu 112"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Sub Menu 113"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Sub Menu 113"))'));

      // Since this is a leaf off of a vertical menu, moving right should
      // return to this menu's parent button.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      expect(focusedMenu, equals('SubmenuButton(Text("Sub Menu 11"))'));

      // Moving left while in a first-level submenu should focus the
      // previous top-level menubar anchor.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 0"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 1"))'));

      // Pressing arrowup from a top-level menubar anchor should focus the last
      // item in that anchor's submenu.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 12"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Sub Menu 11"))'));

      // Enter the submenu.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Sub Menu 110"))'));

      // Move to next top-level menu button.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 2"))'));
    });

    testWidgets('MenuAnchor tab traversal works', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/144381
      final FocusNode buttonFocusNode = FocusNode(debugLabel: TestMenu.anchorButton.label);
      addTearDown(buttonFocusNode.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Column(
              children: <Widget>[
                MenuAnchor(
                  childFocusNode: buttonFocusNode,
                  menuChildren: <Widget>[
                    MenuItemButton(onPressed: () {}, child: const Text('start')),
                    ...createTestMenus(onPressed: onPressed, onOpen: onOpen, onClose: onClose),
                  ],
                  builder: (BuildContext context, MenuController controller, Widget? child) {
                    return TextButton(
                      focusNode: buttonFocusNode,
                      onPressed: () {
                        if (controller.isOpen) {
                          controller.close();
                        } else {
                          controller.open();
                        }
                      },
                      child: Text(TestMenu.anchorButton.label),
                    );
                  },
                ),
              ],
            ),
          ),
        ),
      );

      listenForFocusChanges();

      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pump();
      expect(focusedMenu, equals(TestMenu.anchorButton.label));

      await tester.sendKeyEvent(LogicalKeyboardKey.enter);
      await tester.pump();
      expect(focusedMenu, equals(TestMenu.anchorButton.label));

      // Directional traversal doesn't work until a menu item is focused.
      // To start focusing, hover over the first menu item.
      await hoverOver(tester, find.text('start'));
      await tester.pump();
      expect(focusedMenu, equals('MenuItemButton(Text("start"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 0"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 1"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 2"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pump();
      expect(focusedMenu, equals('MenuItemButton(Text("start"))'));

      await tester.sendKeyDownEvent(LogicalKeyboardKey.shiftLeft);
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 2"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 1"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 0"))'));

      await tester.sendKeyUpEvent(LogicalKeyboardKey.shiftLeft);
      opened.clear();
      closed.clear();

      // Test closing a menu with enter.
      await tester.sendKeyEvent(LogicalKeyboardKey.enter);
      await tester.pump();
      expect(opened, isEmpty);
      expect(closed, <TestMenu>[TestMenu.mainMenu0]);
    });

    testWidgets('MenuAnchor LTR directional traversal works', (WidgetTester tester) async {
      final FocusNode buttonFocusNode = FocusNode(debugLabel: TestMenu.anchorButton.label);
      addTearDown(buttonFocusNode.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Column(
              children: <Widget>[
                MenuAnchor(
                  childFocusNode: buttonFocusNode,
                  menuChildren: <Widget>[
                    MenuItemButton(onPressed: () {}, child: const Text('start')),
                    ...createTestMenus(onPressed: onPressed, onOpen: onOpen, onClose: onClose),
                  ],
                  builder: (BuildContext context, MenuController controller, Widget? child) {
                    return TextButton(
                      focusNode: buttonFocusNode,
                      onPressed: () {
                        if (controller.isOpen) {
                          controller.close();
                        } else {
                          controller.open();
                        }
                      },
                      child: const Text('Open'),
                    );
                  },
                ),
              ],
            ),
          ),
        ),
      );

      listenForFocusChanges();

      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      expect(focusedMenu, equals(TestMenu.anchorButton.label));

      await tester.sendKeyEvent(LogicalKeyboardKey.enter);
      await tester.pump();
      expect(focusedMenu, equals(TestMenu.anchorButton.label));
      expect(find.text('start'), findsOneWidget);

      // Directional traversal doesn't work until a menu item is focused.
      // To start focusing, hover over the first menu item.
      await hoverOver(tester, find.text('start'));
      await tester.pump();
      expect(focusedMenu, equals('MenuItemButton(Text("start"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 0"))'));
      expect(find.text('Sub Menu 00'), findsOne);

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 0"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pump();
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 00"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 01"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 02"))'));

      // We're at the deepest menu on a LTR menu, so arrow right should not change focus.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 02"))'));

      // Arrow left should move focus to the parent anchor.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 0"))'));
      expect(find.text('Sub Menu 00'), findsNothing);

      // We're at the root menu, so arrow left should not change focus and
      // should not open the submenu.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 0"))'));
      expect(find.text('Sub Menu 00'), findsNothing);

      // Open the submenu again.
      await tester.sendKeyEvent(LogicalKeyboardKey.space);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 0"))'));
      expect(find.text('Sub Menu 00'), findsOne);

      // Close all menus.
      await tester.sendKeyEvent(LogicalKeyboardKey.escape);
      await tester.pump();
      expect(focusedMenu, equals(TestMenu.anchorButton.label));
      expect(find.byType(MenuItemButton), findsNothing);
    });

    testWidgets('MenuAnchor RTL directional traversal works', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/119532
      final FocusNode buttonFocusNode = FocusNode(debugLabel: TestMenu.anchorButton.label);
      addTearDown(buttonFocusNode.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Directionality(
            textDirection: TextDirection.rtl,
            child: Material(
              child: Column(
                children: <Widget>[
                  MenuAnchor(
                    childFocusNode: buttonFocusNode,
                    menuChildren: <Widget>[
                      MenuItemButton(onPressed: () {}, child: const Text('start')),
                      ...createTestMenus(onPressed: onPressed, onOpen: onOpen, onClose: onClose),
                    ],
                    builder: (BuildContext context, MenuController controller, Widget? child) {
                      return TextButton(
                        focusNode: buttonFocusNode,
                        onPressed: () {
                          if (controller.isOpen) {
                            controller.close();
                          } else {
                            controller.open();
                          }
                        },
                        child: const Text('Open'),
                      );
                    },
                  ),
                ],
              ),
            ),
          ),
        ),
      );

      listenForFocusChanges();

      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      expect(focusedMenu, equals(TestMenu.anchorButton.label));

      await tester.sendKeyEvent(LogicalKeyboardKey.enter);
      await tester.pump();
      expect(focusedMenu, equals(TestMenu.anchorButton.label));
      expect(find.text('start'), findsOneWidget);

      // Directional traversal doesn't work until a menu item is focused.
      // To start focusing, hover over the first menu item.
      await hoverOver(tester, find.text('start'));
      await tester.pump();
      expect(focusedMenu, equals('MenuItemButton(Text("start"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 0"))'));
      expect(find.text('Sub Menu 00'), findsOne);

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 0"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      await tester.pump();
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 00"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 01"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 02"))'));

      // We're at the deepest menu on a RTL menu, so arrow left should not change focus.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 02"))'));

      // Arrow right should move focus to the parent anchor.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 0"))'));
      expect(find.text('Sub Menu 00'), findsNothing);

      // We're at the root menu, so arrow right should not change focus and
      // should not open the submenu.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 0"))'));
      expect(find.text('Sub Menu 00'), findsNothing);

      // Open the submenu again.
      await tester.sendKeyEvent(LogicalKeyboardKey.space);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 0"))'));
      expect(find.text('Sub Menu 00'), findsOne);

      // Close all menus.
      await tester.sendKeyEvent(LogicalKeyboardKey.escape);
      await tester.pump();
      expect(focusedMenu, equals(TestMenu.anchorButton.label));
      expect(find.byType(MenuItemButton), findsNothing);
    });

    testWidgets('hover traversal works', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              controller: controller,
              children: createTestMenus(onPressed: onPressed, onOpen: onOpen, onClose: onClose),
            ),
          ),
        ),
      );

      listenForFocusChanges();

      // Hovering when the menu is not yet open does nothing.
      await hoverOver(tester, find.text(TestMenu.mainMenu0.label));
      await tester.pump();
      expect(focusedMenu, isNull);

      // Have to open a menu initially to start things going.
      await tester.tap(find.text(TestMenu.mainMenu0.label));
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 0"))'));

      // Hovering when the menu is already  open does nothing.
      await hoverOver(tester, find.text(TestMenu.mainMenu0.label));
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 0"))'));

      // Hovering over the other main menu items opens them now.
      await hoverOver(tester, find.text(TestMenu.mainMenu2.label));
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 2"))'));

      await hoverOver(tester, find.text(TestMenu.mainMenu1.label));
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 1"))'));

      // Hovering over the menu items focuses them.
      await hoverOver(tester, find.text(TestMenu.subMenu10.label));
      await tester.pump();
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 10"))'));

      await hoverOver(tester, find.text(TestMenu.subMenu11.label));
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Sub Menu 11"))'));

      await hoverOver(tester, find.text(TestMenu.subSubMenu110.label));
      await tester.pump();
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Sub Menu 110"))'));
    });

    testWidgets('hover traversal invalidates directional focus scope data', (
      WidgetTester tester,
    ) async {
      // Regression test for https://github.com/flutter/flutter/issues/150910.
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              controller: controller,
              children: createTestMenus(onPressed: onPressed, onOpen: onOpen, onClose: onClose),
            ),
          ),
        ),
      );

      listenForFocusChanges();

      // Have to open a menu initially to start things going.
      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Menu 1"))'));

      await hoverOver(tester, find.text(TestMenu.subMenu12.label));
      await tester.pump();
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 12"))'));

      // Move pointer to disabled menu.
      await hoverOver(tester, find.text(TestMenu.mainMenu5.label));
      await tester.pump();
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 12"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      await tester.pump();
      expect(focusedMenu, equals('SubmenuButton(Text("Sub Menu 11"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 10"))'));

      await hoverOver(tester, find.text(TestMenu.subMenu12.label));
      await tester.pump();
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 12"))'));

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      expect(focusedMenu, equals('MenuItemButton(Text("Sub Menu 12"))'));
    });

    testWidgets('scrolling does not trigger hover traversal', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/150911.
      final GlobalKey scrolledMenuItemKey = GlobalKey();
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuAnchor(
              style: const MenuStyle(fixedSize: WidgetStatePropertyAll<Size>(Size.fromHeight(200))),
              controller: controller,
              menuChildren: <Widget>[
                for (int i = 0; i < 20; i++)
                  MenuItemButton(
                    key: i == 15 ? scrolledMenuItemKey : null,
                    onPressed: () {},
                    child: Text('Item $i'),
                  ),
              ],
            ),
          ),
        ),
      );

      listenForFocusChanges();

      controller.open();
      await tester.pumpAndSettle();

      await hoverOver(tester, find.text('Item 1'));
      await tester.pump();
      expect(focusedMenu, equals('MenuItemButton(Text("Item 1"))'));

      // Scroll the menu while the pointer is over a menu item. The focus should
      // not change.
      tester.renderObject(find.text('Item 15')).showOnScreen();
      await tester.pumpAndSettle();
      expect(focusedMenu, equals('MenuItemButton(Text("Item 1"))'));

      // Traverse with the keyboard to test that the menu scrolls without hover
      // focus affecting the focused menu.
      for (int i = 2; i < 20; i++) {
        await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
        await tester.pump();
        expect(focusedMenu, equals('MenuItemButton(Text("Item $i"))'));
      }
    });

    testWidgets('menus close on ancestor scroll', (WidgetTester tester) async {
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: SingleChildScrollView(
              controller: scrollController,
              child: Container(
                height: 1000,
                alignment: Alignment.center,
                child: MenuBar(
                  controller: controller,
                  children: createTestMenus(onPressed: onPressed, onOpen: onOpen, onClose: onClose),
                ),
              ),
            ),
          ),
        ),
      );

      await tester.tap(find.text(TestMenu.mainMenu0.label));
      await tester.pump();

      expect(opened, isNotEmpty);
      expect(closed, isEmpty);
      opened.clear();

      scrollController.jumpTo(1000);
      await tester.pump();

      expect(opened, isEmpty);
      expect(closed, isNotEmpty);
    });

    testWidgets('menus do not close on root menu internal scroll', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/122168.
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);
      bool rootOpened = false;

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            menuButtonTheme: MenuButtonThemeData(
              // Increase menu items height to make root menu scrollable.
              style: TextButton.styleFrom(minimumSize: const Size.fromHeight(200)),
            ),
          ),
          home: Material(
            child: SingleChildScrollView(
              controller: scrollController,
              child: Container(
                height: 1000,
                alignment: Alignment.topLeft,
                child: MenuAnchor(
                  controller: controller,
                  alignmentOffset: const Offset(0, 10),
                  builder: (BuildContext context, MenuController controller, Widget? child) {
                    return FilledButton.tonal(
                      onPressed: () {
                        if (controller.isOpen) {
                          controller.close();
                        } else {
                          controller.open();
                        }
                      },
                      child: const Text('Show menu'),
                    );
                  },
                  onOpen: () {
                    rootOpened = true;
                  },
                  onClose: () {
                    rootOpened = false;
                  },
                  menuChildren: createTestMenus(
                    onPressed: onPressed,
                    onOpen: onOpen,
                    onClose: onClose,
                    includeExtraGroups: true,
                  ),
                ),
              ),
            ),
          ),
        ),
      );

      await tester.tap(find.text('Show menu'));
      await tester.pump();
      expect(rootOpened, true);

      // Hover the first item.
      final TestPointer pointer = TestPointer(1, PointerDeviceKind.mouse);
      await tester.sendEventToBinding(
        pointer.hover(tester.getCenter(find.text(TestMenu.mainMenu0.label))),
      );
      await tester.pump();
      expect(opened, isNotEmpty);

      // Menus do not close on internal scroll.
      await tester.sendEventToBinding(pointer.scroll(const Offset(0.0, 30.0)));
      await tester.pump();
      expect(rootOpened, true);
      expect(closed, isEmpty);

      // Menus close on external scroll.
      scrollController.jumpTo(1000);
      await tester.pump();
      expect(rootOpened, false);
      expect(closed, isNotEmpty);
    });

    testWidgets('menus close on view size change', (WidgetTester tester) async {
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);
      final MediaQueryData mediaQueryData = MediaQueryData.fromView(tester.view);

      Widget build(Size size) {
        return MaterialApp(
          home: Material(
            child: MediaQuery(
              data: mediaQueryData.copyWith(size: size),
              child: SingleChildScrollView(
                controller: scrollController,
                child: Container(
                  height: 1000,
                  alignment: Alignment.center,
                  child: MenuBar(
                    controller: controller,
                    children: createTestMenus(
                      onPressed: onPressed,
                      onOpen: onOpen,
                      onClose: onClose,
                    ),
                  ),
                ),
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(build(mediaQueryData.size));

      await tester.tap(find.text(TestMenu.mainMenu0.label));
      await tester.pump();

      expect(opened, isNotEmpty);
      expect(closed, isEmpty);
      opened.clear();

      const Size smallSize = Size(200, 200);
      await changeSurfaceSize(tester, smallSize);

      await tester.pumpWidget(build(smallSize));
      await tester.pump();

      expect(opened, isEmpty);
      expect(closed, isNotEmpty);
    });

    // Regression test for
    // https://github.com/flutter/flutter/issues/119532#issuecomment-2274705565.
    testWidgets('Shortcuts of MenuAnchor do not rely on WidgetsApp.shortcuts', (
      WidgetTester tester,
    ) async {
      // MenuAnchor used to rely on WidgetsApp.shortcuts for menu navigation,
      // which is a problem for Web because the Web uses a special set of
      // default shortcuts that define arrow keys as scrolling instead of
      // traversing, and therefore arrow keys won't enter submenus when the
      // focus is on MenuAnchor.
      //
      // This test verifies that `MenuAnchor`'s shortcuts continues to work even
      // when `WidgetsApp.shortcuts` contains nothing.

      final FocusNode childNode = FocusNode(debugLabel: 'Dropdown Inkwell');
      addTearDown(childNode.dispose);

      await tester.pumpWidget(
        MaterialApp(
          // Clear WidgetsApp.shortcuts to make sure MenuAnchor doesn't rely on
          // it.
          shortcuts: const <ShortcutActivator, Intent>{},
          home: Scaffold(
            body: MenuAnchor(
              childFocusNode: childNode,
              menuChildren: List<Widget>.generate(
                3,
                (int i) => MenuItemButton(child: Text('Submenu item $i'), onPressed: () {}),
              ),
              builder: (BuildContext context, MenuController controller, Widget? child) {
                return InkWell(
                  focusNode: childNode,
                  onTap: controller.open,
                  child: const Text('Main button'),
                );
              },
            ),
          ),
        ),
      );

      listenForFocusChanges();

      // Open the drop down menu and focus on the MenuAnchor.
      await tester.tap(find.text('Main button'));
      await tester.pumpAndSettle();
      expect(find.text('Submenu item 0'), findsOneWidget);

      // Press arrowDown, and the first submenu button should be focused.
      // This is the critical part. It used to not work on Web.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.pump();
      expect(focusedMenu, equals('MenuItemButton(Text("Submenu item 0"))'));

      // Press arrowDown, and the second submenu button should be focused.
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.pump();
      expect(focusedMenu, equals('MenuItemButton(Text("Submenu item 1"))'));
    });
  });

  group('Accelerators', () {
    const Set<TargetPlatform> apple = <TargetPlatform>{TargetPlatform.macOS, TargetPlatform.iOS};
    final Set<TargetPlatform> nonApple = TargetPlatform.values.toSet().difference(apple);

    test('Accelerator markers are stripped properly', () {
      const Map<String, String> expected = <String, String>{
        'Plain String': 'Plain String',
        '&Simple Accelerator': 'Simple Accelerator',
        '&Multiple &Accelerators': 'Multiple Accelerators',
        'Whitespace & Accelerators': 'Whitespace  Accelerators',
        '&Quoted && Ampersand': 'Quoted & Ampersand',
        'Ampersand at End &': 'Ampersand at End ',
        '&&Multiple Ampersands &&& &&&A &&&&B &&&&': '&Multiple Ampersands & &A &&B &&',
        'Bohrium 𨨏 Code point U+28A0F': 'Bohrium 𨨏 Code point U+28A0F',
      };
      const List<int> expectedIndices = <int>[-1, 0, 0, -1, 0, -1, 24, -1];
      const List<bool> expectedHasAccelerator = <bool>[
        false,
        true,
        true,
        false,
        true,
        false,
        true,
        false,
      ];
      int acceleratorIndex = -1;
      int count = 0;
      for (final String key in expected.keys) {
        expect(
          MenuAcceleratorLabel.stripAcceleratorMarkers(
            key,
            setIndex: (int index) {
              acceleratorIndex = index;
            },
          ),
          equals(expected[key]),
          reason: "'$key' label doesn't match ${expected[key]}",
        );
        expect(
          acceleratorIndex,
          equals(expectedIndices[count]),
          reason: "'$key' index doesn't match ${expectedIndices[count]}",
        );
        expect(
          MenuAcceleratorLabel(key).hasAccelerator,
          equals(expectedHasAccelerator[count]),
          reason: "'$key' hasAccelerator isn't ${expectedHasAccelerator[count]}",
        );
        count += 1;
      }
    });

    testWidgets('can invoke menu items', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              key: UniqueKey(),
              controller: controller,
              children: createTestMenus(
                onPressed: onPressed,
                onOpen: onOpen,
                onClose: onClose,
                accelerators: true,
              ),
            ),
          ),
        ),
      );

      await tester.sendKeyDownEvent(LogicalKeyboardKey.altLeft);
      await tester.pump();
      await tester.sendKeyEvent(LogicalKeyboardKey.keyM, character: 'm');
      await tester.pump();
      // Makes sure that identical accelerators in parent menu items don't
      // shadow the ones in the children.
      await tester.sendKeyEvent(LogicalKeyboardKey.keyM, character: 'm');
      await tester.sendKeyUpEvent(LogicalKeyboardKey.altLeft);
      await tester.pump();

      expect(opened, equals(<TestMenu>[TestMenu.mainMenu0]));
      expect(closed, equals(<TestMenu>[TestMenu.mainMenu0]));
      expect(selected, equals(<TestMenu>[TestMenu.subMenu00]));
      // Selecting a non-submenu item should close all the menus.
      expect(find.text(TestMenu.subMenu00.label), findsNothing);
      opened.clear();
      closed.clear();
      selected.clear();

      // Invoking several levels deep.
      await tester.sendKeyDownEvent(LogicalKeyboardKey.altRight);
      await tester.pump();
      await tester.sendKeyEvent(LogicalKeyboardKey.keyM, character: 'e');
      await tester.pump();
      await tester.sendKeyEvent(LogicalKeyboardKey.keyM, character: '1');
      await tester.pump();
      await tester.sendKeyEvent(LogicalKeyboardKey.keyM, character: '1');
      await tester.sendKeyUpEvent(LogicalKeyboardKey.altRight);
      await tester.pump();

      expect(opened, equals(<TestMenu>[TestMenu.mainMenu1, TestMenu.subMenu11]));
      expect(closed, equals(<TestMenu>[TestMenu.subMenu11, TestMenu.mainMenu1]));
      expect(selected, equals(<TestMenu>[TestMenu.subSubMenu111]));
      opened.clear();
      closed.clear();
      selected.clear();
    }, variant: TargetPlatformVariant(nonApple));

    testWidgets('can combine with regular keyboard navigation', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              key: UniqueKey(),
              controller: controller,
              children: createTestMenus(
                onPressed: onPressed,
                onOpen: onOpen,
                onClose: onClose,
                accelerators: true,
              ),
            ),
          ),
        ),
      );

      // Combining accelerators and regular keyboard navigation works.
      await tester.sendKeyDownEvent(LogicalKeyboardKey.altLeft);
      await tester.pump();
      await tester.sendKeyEvent(LogicalKeyboardKey.keyM, character: 'e');
      await tester.pump();
      await tester.sendKeyEvent(LogicalKeyboardKey.keyM, character: '1');
      await tester.pump();
      await tester.sendKeyUpEvent(LogicalKeyboardKey.altLeft);
      await tester.pump();
      await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
      await tester.pump();
      await tester.sendKeyEvent(LogicalKeyboardKey.enter);
      await tester.pump();

      expect(opened, equals(<TestMenu>[TestMenu.mainMenu1, TestMenu.subMenu11]));
      expect(closed, equals(<TestMenu>[TestMenu.subMenu11, TestMenu.mainMenu1]));
      expect(selected, equals(<TestMenu>[TestMenu.subSubMenu110]));
    }, variant: TargetPlatformVariant(nonApple));

    testWidgets('can combine with mouse', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              key: UniqueKey(),
              controller: controller,
              children: createTestMenus(
                onPressed: onPressed,
                onOpen: onOpen,
                onClose: onClose,
                accelerators: true,
              ),
            ),
          ),
        ),
      );

      // Combining accelerators and regular keyboard navigation works.
      await tester.sendKeyDownEvent(LogicalKeyboardKey.altLeft);
      await tester.pump();
      await tester.sendKeyEvent(LogicalKeyboardKey.keyM, character: 'e');
      await tester.pump();
      await tester.sendKeyEvent(LogicalKeyboardKey.keyM, character: '1');
      await tester.pump();
      await tester.sendKeyUpEvent(LogicalKeyboardKey.altLeft);
      await tester.pump();
      await tester.tap(find.text(TestMenu.subSubMenu112.label));
      await tester.pump();

      expect(opened, equals(<TestMenu>[TestMenu.mainMenu1, TestMenu.subMenu11]));
      expect(closed, equals(<TestMenu>[TestMenu.subMenu11, TestMenu.mainMenu1]));
      expect(selected, equals(<TestMenu>[TestMenu.subSubMenu112]));
    }, variant: TargetPlatformVariant(nonApple));

    testWidgets("disabled items don't respond to accelerators", (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              key: UniqueKey(),
              controller: controller,
              children: createTestMenus(
                onPressed: onPressed,
                onOpen: onOpen,
                onClose: onClose,
                accelerators: true,
              ),
            ),
          ),
        ),
      );

      await tester.sendKeyDownEvent(LogicalKeyboardKey.altLeft);
      await tester.pump();
      await tester.sendKeyEvent(LogicalKeyboardKey.keyM, character: '5');
      await tester.pump();
      await tester.sendKeyUpEvent(LogicalKeyboardKey.altLeft);
      await tester.pump();

      expect(opened, isEmpty);
      expect(closed, isEmpty);
      expect(selected, isEmpty);
      // Selecting a non-submenu item should close all the menus.
      expect(find.text(TestMenu.subMenu00.label), findsNothing);
    }, variant: TargetPlatformVariant(nonApple));

    testWidgets("Apple platforms don't react to accelerators", (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              key: UniqueKey(),
              controller: controller,
              children: createTestMenus(
                onPressed: onPressed,
                onOpen: onOpen,
                onClose: onClose,
                accelerators: true,
              ),
            ),
          ),
        ),
      );

      await tester.sendKeyDownEvent(LogicalKeyboardKey.altLeft);
      await tester.pump();
      await tester.sendKeyEvent(LogicalKeyboardKey.keyM, character: 'm');
      await tester.pump();
      await tester.sendKeyEvent(LogicalKeyboardKey.keyM, character: 'm');
      await tester.sendKeyUpEvent(LogicalKeyboardKey.altLeft);
      await tester.pump();

      expect(opened, isEmpty);
      expect(closed, isEmpty);
      expect(selected, isEmpty);

      // Or with the option key equivalents.
      await tester.sendKeyDownEvent(LogicalKeyboardKey.altLeft);
      await tester.pump();
      await tester.sendKeyEvent(LogicalKeyboardKey.keyM, character: 'µ');
      await tester.pump();
      await tester.sendKeyEvent(LogicalKeyboardKey.keyM, character: 'µ');
      await tester.sendKeyUpEvent(LogicalKeyboardKey.altLeft);
      await tester.pump();

      expect(opened, isEmpty);
      expect(closed, isEmpty);
      expect(selected, isEmpty);
    }, variant: const TargetPlatformVariant(apple));
  });

  group('MenuController', () {
    testWidgets('Moving a controller to a new instance works', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(key: UniqueKey(), controller: controller, children: createTestMenus()),
          ),
        ),
      );

      // Open a menu initially.
      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();

      await tester.tap(find.text(TestMenu.subMenu11.label));
      await tester.pump();

      // Now pump a new menu with a different UniqueKey to dispose of the opened
      // menu's node, but keep the existing controller.
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              key: UniqueKey(),
              controller: controller,
              children: createTestMenus(includeExtraGroups: true),
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();
    });

    testWidgets('closing via controller works', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              controller: controller,
              children: createTestMenus(
                onPressed: onPressed,
                onOpen: onOpen,
                onClose: onClose,
                shortcuts: <TestMenu, MenuSerializableShortcut>{
                  TestMenu.subSubMenu110: const SingleActivator(
                    LogicalKeyboardKey.keyA,
                    control: true,
                  ),
                },
              ),
            ),
          ),
        ),
      );

      // Open a menu initially.
      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();

      await tester.tap(find.text(TestMenu.subMenu11.label));
      await tester.pump();
      expect(opened, unorderedEquals(<TestMenu>[TestMenu.mainMenu1, TestMenu.subMenu11]));
      opened.clear();
      closed.clear();

      // Close menus using the controller.
      controller.close();
      await tester.pump();

      // The menu should go away,
      expect(closed, unorderedEquals(<TestMenu>[TestMenu.mainMenu1, TestMenu.subMenu11]));
      expect(opened, isEmpty);
    });
  });

  group('MenuItemButton', () {
    testWidgets('Shortcut mnemonics are displayed', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              controller: controller,
              children: createTestMenus(
                shortcuts: <TestMenu, MenuSerializableShortcut>{
                  TestMenu.subSubMenu110: const SingleActivator(
                    LogicalKeyboardKey.keyA,
                    control: true,
                  ),
                  TestMenu.subSubMenu111: const SingleActivator(
                    LogicalKeyboardKey.keyB,
                    shift: true,
                  ),
                  TestMenu.subSubMenu112: const SingleActivator(LogicalKeyboardKey.keyC, alt: true),
                  TestMenu.subSubMenu113: const SingleActivator(
                    LogicalKeyboardKey.keyD,
                    meta: true,
                  ),
                },
              ),
            ),
          ),
        ),
      );

      // Open a menu initially.
      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();

      await tester.tap(find.text(TestMenu.subMenu11.label));
      await tester.pump();

      Text mnemonic0 = tester.widget(findMnemonic(TestMenu.subSubMenu110.label));
      Text mnemonic1 = tester.widget(findMnemonic(TestMenu.subSubMenu111.label));
      Text mnemonic2 = tester.widget(findMnemonic(TestMenu.subSubMenu112.label));
      Text mnemonic3 = tester.widget(findMnemonic(TestMenu.subSubMenu113.label));

      switch (defaultTargetPlatform) {
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
          expect(mnemonic0.data, equals('Ctrl+A'));
          expect(mnemonic1.data, equals('Shift+B'));
          expect(mnemonic2.data, equals('Alt+C'));
          expect(mnemonic3.data, equals('Meta+D'));
        case TargetPlatform.windows:
          expect(mnemonic0.data, equals('Ctrl+A'));
          expect(mnemonic1.data, equals('Shift+B'));
          expect(mnemonic2.data, equals('Alt+C'));
          expect(mnemonic3.data, equals('Win+D'));
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          expect(mnemonic0.data, equals('⌃ A'));
          expect(mnemonic1.data, equals('⇧ B'));
          expect(mnemonic2.data, equals('⌥ C'));
          expect(mnemonic3.data, equals('⌘ D'));
      }

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              controller: controller,
              children: createTestMenus(
                includeExtraGroups: true,
                shortcuts: <TestMenu, MenuSerializableShortcut>{
                  TestMenu.subSubMenu110: const SingleActivator(LogicalKeyboardKey.arrowRight),
                  TestMenu.subSubMenu111: const SingleActivator(LogicalKeyboardKey.arrowLeft),
                  TestMenu.subSubMenu112: const SingleActivator(LogicalKeyboardKey.arrowUp),
                  TestMenu.subSubMenu113: const SingleActivator(LogicalKeyboardKey.arrowDown),
                },
              ),
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();

      mnemonic0 = tester.widget(findMnemonic(TestMenu.subSubMenu110.label));
      expect(mnemonic0.data, equals('→'));
      mnemonic1 = tester.widget(findMnemonic(TestMenu.subSubMenu111.label));
      expect(mnemonic1.data, equals('←'));
      mnemonic2 = tester.widget(findMnemonic(TestMenu.subSubMenu112.label));
      expect(mnemonic2.data, equals('↑'));
      mnemonic3 = tester.widget(findMnemonic(TestMenu.subSubMenu113.label));
      expect(mnemonic3.data, equals('↓'));

      // Try some weirder ones.
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              controller: controller,
              children: createTestMenus(
                shortcuts: <TestMenu, MenuSerializableShortcut>{
                  TestMenu.subSubMenu110: const SingleActivator(LogicalKeyboardKey.escape),
                  TestMenu.subSubMenu111: const SingleActivator(LogicalKeyboardKey.fn),
                  TestMenu.subSubMenu112: const SingleActivator(LogicalKeyboardKey.enter),
                },
              ),
            ),
          ),
        ),
      );
      await tester.pumpAndSettle();

      mnemonic0 = tester.widget(findMnemonic(TestMenu.subSubMenu110.label));
      expect(mnemonic0.data, equals('Esc'));
      mnemonic1 = tester.widget(findMnemonic(TestMenu.subSubMenu111.label));
      expect(mnemonic1.data, equals('Fn'));
      mnemonic2 = tester.widget(findMnemonic(TestMenu.subSubMenu112.label));
      expect(mnemonic2.data, equals('↵'));
    }, variant: TargetPlatformVariant.all());

    // Regression test for https://github.com/flutter/flutter/issues/145040.
    testWidgets('CharacterActivator shortcut mnemonics include modifiers', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              controller: controller,
              children: createTestMenus(
                shortcuts: <TestMenu, MenuSerializableShortcut>{
                  TestMenu.subSubMenu110: const CharacterActivator('A', control: true),
                  TestMenu.subSubMenu111: const CharacterActivator('B', alt: true),
                  TestMenu.subSubMenu112: const CharacterActivator('C', meta: true),
                },
              ),
            ),
          ),
        ),
      );

      // Open a menu initially.
      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();

      await tester.tap(find.text(TestMenu.subMenu11.label));
      await tester.pump();

      final Text mnemonic0 = tester.widget(findMnemonic(TestMenu.subSubMenu110.label));
      final Text mnemonic1 = tester.widget(findMnemonic(TestMenu.subSubMenu111.label));
      final Text mnemonic2 = tester.widget(findMnemonic(TestMenu.subSubMenu112.label));

      switch (defaultTargetPlatform) {
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
          expect(mnemonic0.data, equals('Ctrl+A'));
          expect(mnemonic1.data, equals('Alt+B'));
          expect(mnemonic2.data, equals('Meta+C'));
        case TargetPlatform.windows:
          expect(mnemonic0.data, equals('Ctrl+A'));
          expect(mnemonic1.data, equals('Alt+B'));
          expect(mnemonic2.data, equals('Win+C'));
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          expect(mnemonic0.data, equals('⌃ A'));
          expect(mnemonic1.data, equals('⌥ B'));
          expect(mnemonic2.data, equals('⌘ C'));
      }
    }, variant: TargetPlatformVariant.all());

    testWidgets('leadingIcon is used when set', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              controller: controller,
              children: <Widget>[
                SubmenuButton(
                  menuChildren: <Widget>[
                    MenuItemButton(
                      leadingIcon: const Text('leadingIcon'),
                      child: Text(TestMenu.subMenu00.label),
                    ),
                  ],
                  child: Text(TestMenu.mainMenu0.label),
                ),
              ],
            ),
          ),
        ),
      );

      await tester.tap(find.text(TestMenu.mainMenu0.label));
      await tester.pump();

      expect(find.text('leadingIcon'), findsOneWidget);
    });

    testWidgets('autofocus is used when set and widget is enabled', (WidgetTester tester) async {
      listenForFocusChanges();

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Column(
              children: <Widget>[
                MenuAnchor(
                  controller: controller,
                  menuChildren: <Widget>[
                    MenuItemButton(
                      autofocus: true,
                      // Required for clickability.
                      onPressed: () {},
                      child: Text(TestMenu.mainMenu0.label),
                    ),
                    MenuItemButton(onPressed: () {}, child: Text(TestMenu.mainMenu1.label)),
                  ],
                ),
                const Expanded(child: Placeholder()),
              ],
            ),
          ),
        ),
      );

      controller.open();
      await tester.pump();

      expect(controller.isOpen, equals(true));
      expect(focusedMenu, equals('MenuItemButton(Text("${TestMenu.mainMenu0.label}"))'));
    });

    testWidgets('trailingIcon is used when set', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              controller: controller,
              children: <Widget>[
                SubmenuButton(
                  menuChildren: <Widget>[
                    MenuItemButton(
                      trailingIcon: const Text('trailingIcon'),
                      child: Text(TestMenu.subMenu00.label),
                    ),
                  ],
                  child: Text(TestMenu.mainMenu0.label),
                ),
              ],
            ),
          ),
        ),
      );

      await tester.tap(find.text(TestMenu.mainMenu0.label));
      await tester.pump();

      expect(find.text('trailingIcon'), findsOneWidget);
    });

    testWidgets('SubmenuButton uses supplied controller', (WidgetTester tester) async {
      final MenuController submenuController = MenuController();
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              controller: controller,
              children: <Widget>[
                SubmenuButton(
                  controller: submenuController,
                  menuChildren: <Widget>[MenuItemButton(child: Text(TestMenu.subMenu00.label))],
                  child: Text(TestMenu.mainMenu0.label),
                ),
              ],
            ),
          ),
        ),
      );

      submenuController.open();
      await tester.pump();
      expect(find.text(TestMenu.subMenu00.label), findsOneWidget);

      submenuController.close();
      await tester.pump();
      expect(find.text(TestMenu.subMenu00.label), findsNothing);

      // Now remove the controller and try to control it.
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              controller: controller,
              children: <Widget>[
                SubmenuButton(
                  menuChildren: <Widget>[MenuItemButton(child: Text(TestMenu.subMenu00.label))],
                  child: Text(TestMenu.mainMenu0.label),
                ),
              ],
            ),
          ),
        ),
      );

      await expectLater(() => submenuController.open(), throwsAssertionError);
      await tester.pump();
      expect(find.text(TestMenu.subMenu00.label), findsNothing);
    });

    testWidgets('diagnostics', (WidgetTester tester) async {
      final ButtonStyle style = ButtonStyle(
        shape: MaterialStateProperty.all<OutlinedBorder?>(const StadiumBorder()),
        elevation: MaterialStateProperty.all<double?>(10.0),
        backgroundColor: const MaterialStatePropertyAll<Color>(Colors.red),
      );
      final MenuStyle menuStyle = MenuStyle(
        shape: MaterialStateProperty.all<OutlinedBorder?>(const RoundedRectangleBorder()),
        elevation: MaterialStateProperty.all<double?>(20.0),
        backgroundColor: const MaterialStatePropertyAll<Color>(Colors.green),
      );
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              controller: controller,
              children: <Widget>[
                SubmenuButton(
                  style: style,
                  menuStyle: menuStyle,
                  menuChildren: <Widget>[
                    MenuItemButton(style: style, child: Text(TestMenu.subMenu00.label)),
                  ],
                  child: Text(TestMenu.mainMenu0.label),
                ),
              ],
            ),
          ),
        ),
      );

      await tester.tap(find.text(TestMenu.mainMenu0.label));
      await tester.pump();

      final SubmenuButton submenu = tester.widget(find.byType(SubmenuButton));
      final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
      submenu.debugFillProperties(builder);

      final List<String> description =
          builder.properties
              .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
              .map((DiagnosticsNode node) => node.toString())
              .toList();

      expect(
        description,
        equalsIgnoringHashCodes(<String>[
          'focusNode: null',
          'menuStyle: MenuStyle#00000(backgroundColor: WidgetStatePropertyAll(MaterialColor(primary value: ${const Color(0xff4caf50)})), elevation: WidgetStatePropertyAll(20.0), shape: WidgetStatePropertyAll(RoundedRectangleBorder(BorderSide(width: 0.0, style: none), BorderRadius.zero)))',
          'alignmentOffset: null',
          'clipBehavior: hardEdge',
        ]),
      );
    });

    testWidgets('MenuItemButton respects closeOnActivate property', (WidgetTester tester) async {
      final MenuController controller = MenuController();
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: MenuAnchor(
                controller: controller,
                menuChildren: <Widget>[
                  MenuItemButton(onPressed: () {}, child: const Text('Button 1')),
                ],
                builder: (BuildContext context, MenuController controller, Widget? child) {
                  return FilledButton(
                    onPressed: () {
                      controller.open();
                    },
                    child: const Text('Tap me'),
                  );
                },
              ),
            ),
          ),
        ),
      );

      await tester.tap(find.text('Tap me'));
      await tester.pump();
      expect(find.byType(MenuItemButton), findsNWidgets(1));

      // Taps the MenuItemButton which should close the menu.
      await tester.tap(find.text('Button 1'));
      await tester.pump();
      expect(find.byType(MenuItemButton), findsNWidgets(0));

      await tester.pumpAndSettle();

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: MenuAnchor(
                controller: controller,
                menuChildren: <Widget>[
                  MenuItemButton(
                    closeOnActivate: false,
                    onPressed: () {},
                    child: const Text('Button 1'),
                  ),
                ],
                builder: (BuildContext context, MenuController controller, Widget? child) {
                  return FilledButton(
                    onPressed: () {
                      controller.open();
                    },
                    child: const Text('Tap me'),
                  );
                },
              ),
            ),
          ),
        ),
      );

      await tester.tap(find.text('Tap me'));
      await tester.pump();
      expect(find.byType(MenuItemButton), findsNWidgets(1));

      // Taps the MenuItemButton which shouldn't close the menu.
      await tester.tap(find.text('Button 1'));
      await tester.pump();
      expect(find.byType(MenuItemButton), findsNWidgets(1));
    });

    // This is a regression test for https://github.com/flutter/flutter/issues/129439.
    testWidgets('MenuItemButton does not overflow when child is long', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: SizedBox(
              width: 200,
              child: MenuItemButton(
                overflowAxis: Axis.vertical,
                onPressed: () {},
                child: const Text('MenuItem Button does not overflow when child is long'),
              ),
            ),
          ),
        ),
      );

      // No exception should be thrown.
      expect(tester.takeException(), isNull);
    });

    testWidgets('MenuItemButton layout is updated by overflowAxis', (WidgetTester tester) async {
      Widget buildMenuButton({required Axis overflowAxis, bool constrainedLayout = false}) {
        return MaterialApp(
          home: Scaffold(
            body: SizedBox(
              width: constrainedLayout ? 200 : null,
              child: MenuItemButton(
                overflowAxis: overflowAxis,
                onPressed: () {},
                child: const Text('This is a very long text that will wrap to the multiple lines.'),
              ),
            ),
          ),
        );
      }

      // Test a long MenuItemButton in an unconstrained layout with vertical overflow axis.
      await tester.pumpWidget(buildMenuButton(overflowAxis: Axis.vertical));
      expect(tester.getSize(find.byType(MenuItemButton)), const Size(800.0, 48.0));

      // Test a long MenuItemButton in an unconstrained layout with horizontal overflow axis.
      await tester.pumpWidget(buildMenuButton(overflowAxis: Axis.horizontal));
      expect(tester.getSize(find.byType(MenuItemButton)), const Size(800.0, 48.0));

      // Test a long MenuItemButton in a constrained layout with vertical overflow axis.
      await tester.pumpWidget(
        buildMenuButton(overflowAxis: Axis.vertical, constrainedLayout: true),
      );
      expect(tester.getSize(find.byType(MenuItemButton)), const Size(200.0, 120.0));

      // Test a long MenuItemButton in a constrained layout with horizontal overflow axis.
      await tester.pumpWidget(
        buildMenuButton(overflowAxis: Axis.horizontal, constrainedLayout: true),
      );
      expect(tester.getSize(find.byType(MenuItemButton)), const Size(200.0, 48.0));
      // This should throw an error.
      final AssertionError exception = tester.takeException() as AssertionError;
      expect(exception, isAssertionError);
    });

    testWidgets('MenuItemButton.styleFrom overlayColor overrides default overlay color', (
      WidgetTester tester,
    ) async {
      const Color overlayColor = Color(0xffff0000);
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: MenuItemButton(
              style: MenuItemButton.styleFrom(overlayColor: overlayColor),
              onPressed: () {},
              child: const Text('MenuItem'),
            ),
          ),
        ),
      );

      // Hovered.
      final Offset center = tester.getCenter(find.byType(MenuItemButton));
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
          ..rect(color: overlayColor.withOpacity(0.08))
          ..rect(color: overlayColor.withOpacity(0.1)),
      );
    });

    // Regression test for https://github.com/flutter/flutter/issues/147479.
    testWidgets('MenuItemButton can build when its child is null', (WidgetTester tester) async {
      await tester.pumpWidget(
        const MaterialApp(home: Scaffold(body: SizedBox(width: 200, child: MenuItemButton()))),
      );

      expect(tester.takeException(), isNull);
    });
  });

  group('Layout', () {
    List<Rect> collectMenuItemRects() {
      final List<Rect> menuRects = <Rect>[];
      final List<Element> candidates = find.byType(SubmenuButton).evaluate().toList();
      for (final Element candidate in candidates) {
        final RenderBox box = candidate.renderObject! as RenderBox;
        final Offset topLeft = box.localToGlobal(box.size.topLeft(Offset.zero));
        final Offset bottomRight = box.localToGlobal(box.size.bottomRight(Offset.zero));
        menuRects.add(Rect.fromPoints(topLeft, bottomRight));
      }
      return menuRects;
    }

    List<Rect> collectSubmenuRects() {
      final List<Rect> menuRects = <Rect>[];
      final List<Element> candidates = findMenuPanels().evaluate().toList();
      for (final Element candidate in candidates) {
        final RenderBox box = candidate.renderObject! as RenderBox;
        final Offset topLeft = box.localToGlobal(box.size.topLeft(Offset.zero));
        final Offset bottomRight = box.localToGlobal(box.size.bottomRight(Offset.zero));
        menuRects.add(Rect.fromPoints(topLeft, bottomRight));
      }
      return menuRects;
    }

    testWidgets('unconstrained menus show up in the right place in LTR', (
      WidgetTester tester,
    ) async {
      await changeSurfaceSize(tester, const Size(800, 600));
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(
            child: Column(
              children: <Widget>[
                Row(
                  children: <Widget>[
                    Expanded(child: MenuBar(children: createTestMenus(onPressed: onPressed))),
                  ],
                ),
                const Expanded(child: Placeholder()),
              ],
            ),
          ),
        ),
      );
      await tester.pump();

      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();
      await tester.tap(find.text(TestMenu.subMenu11.label));
      await tester.pump();

      expect(find.byType(MenuItemButton), findsNWidgets(6));
      expect(find.byType(SubmenuButton), findsNWidgets(5));
      expect(
        collectMenuItemRects(),
        equals(const <Rect>[
          Rect.fromLTRB(4.0, 0.0, 112.0, 48.0),
          Rect.fromLTRB(112.0, 0.0, 220.0, 48.0),
          Rect.fromLTRB(112.0, 104.0, 326.0, 152.0),
          Rect.fromLTRB(220.0, 0.0, 328.0, 48.0),
          Rect.fromLTRB(328.0, 0.0, 506.0, 48.0),
        ]),
      );
    });

    testWidgets('unconstrained menus show up in the right place in RTL', (
      WidgetTester tester,
    ) async {
      await changeSurfaceSize(tester, const Size(800, 600));
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Directionality(
            textDirection: TextDirection.rtl,
            child: Material(
              child: Column(
                children: <Widget>[
                  Row(
                    children: <Widget>[
                      Expanded(child: MenuBar(children: createTestMenus(onPressed: onPressed))),
                    ],
                  ),
                  const Expanded(child: Placeholder()),
                ],
              ),
            ),
          ),
        ),
      );
      await tester.pump();

      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();
      await tester.tap(find.text(TestMenu.subMenu11.label));
      await tester.pump();

      expect(find.byType(MenuItemButton), findsNWidgets(6));
      expect(find.byType(SubmenuButton), findsNWidgets(5));
      expect(
        collectMenuItemRects(),
        equals(const <Rect>[
          Rect.fromLTRB(688.0, 0.0, 796.0, 48.0),
          Rect.fromLTRB(580.0, 0.0, 688.0, 48.0),
          Rect.fromLTRB(474.0, 104.0, 688.0, 152.0),
          Rect.fromLTRB(472.0, 0.0, 580.0, 48.0),
          Rect.fromLTRB(294.0, 0.0, 472.0, 48.0),
        ]),
      );
    });

    testWidgets('constrained menus show up in the right place in LTR', (WidgetTester tester) async {
      await changeSurfaceSize(tester, const Size(300, 300));
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Builder(
            builder: (BuildContext context) {
              return Directionality(
                textDirection: TextDirection.ltr,
                child: Material(
                  child: Column(
                    children: <Widget>[
                      MenuBar(children: createTestMenus(onPressed: onPressed)),
                      const Expanded(child: Placeholder()),
                    ],
                  ),
                ),
              );
            },
          ),
        ),
      );
      await tester.pump();

      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();
      await tester.tap(find.text(TestMenu.subMenu11.label));
      await tester.pump();

      expect(find.byType(MenuItemButton), findsNWidgets(6));
      expect(find.byType(SubmenuButton), findsNWidgets(5));
      expect(
        collectMenuItemRects(),
        equals(const <Rect>[
          Rect.fromLTRB(4.0, 0.0, 112.0, 48.0),
          Rect.fromLTRB(112.0, 0.0, 220.0, 48.0),
          Rect.fromLTRB(86.0, 104.0, 300.0, 152.0),
          Rect.fromLTRB(220.0, 0.0, 328.0, 48.0),
          Rect.fromLTRB(328.0, 0.0, 506.0, 48.0),
        ]),
      );
    });

    testWidgets('tapping MenuItemButton with null focus node', (WidgetTester tester) async {
      FocusNode? buttonFocusNode = FocusNode();

      // Build our app and trigger a frame.
      await tester.pumpWidget(
        MaterialApp(
          home: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return MenuAnchor(
                menuChildren: <Widget>[
                  MenuItemButton(
                    focusNode: buttonFocusNode,
                    closeOnActivate: false,
                    child: const Text('Set focus to null'),
                    onPressed: () {
                      setState(() {
                        buttonFocusNode?.dispose();
                        buttonFocusNode = null;
                      });
                    },
                  ),
                ],
                builder: (BuildContext context, MenuController controller, Widget? child) {
                  return TextButton(
                    onPressed: () {
                      if (controller.isOpen) {
                        controller.close();
                      } else {
                        controller.open();
                      }
                    },
                    child: const Text('OPEN MENU'),
                  );
                },
              );
            },
          ),
        ),
      );

      await tester.tap(find.text('OPEN MENU'));
      await tester.pump();

      expect(find.text('Set focus to null'), findsOneWidget);

      await tester.tap(find.text('Set focus to null'));
      await tester.pumpAndSettle();

      expect(tester.takeException(), isNull);
    });

    testWidgets('constrained menus show up in the right place in RTL', (WidgetTester tester) async {
      await changeSurfaceSize(tester, const Size(300, 300));
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Builder(
            builder: (BuildContext context) {
              return Directionality(
                textDirection: TextDirection.rtl,
                child: Material(
                  child: Column(
                    children: <Widget>[
                      MenuBar(children: createTestMenus(onPressed: onPressed)),
                      const Expanded(child: Placeholder()),
                    ],
                  ),
                ),
              );
            },
          ),
        ),
      );
      await tester.pump();
      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();
      await tester.tap(find.text(TestMenu.subMenu11.label));
      await tester.pump();

      expect(find.byType(MenuItemButton), findsNWidgets(6));
      expect(find.byType(SubmenuButton), findsNWidgets(5));
      expect(
        collectMenuItemRects(),
        equals(const <Rect>[
          Rect.fromLTRB(188.0, 0.0, 296.0, 48.0),
          Rect.fromLTRB(80.0, 0.0, 188.0, 48.0),
          Rect.fromLTRB(0.0, 104.0, 214.0, 152.0),
          Rect.fromLTRB(-28.0, 0.0, 80.0, 48.0),
          Rect.fromLTRB(-206.0, 0.0, -28.0, 48.0),
        ]),
      );
    });

    testWidgets('constrained menus show up in the right place with offset in LTR', (
      WidgetTester tester,
    ) async {
      await changeSurfaceSize(tester, const Size(800, 600));
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Builder(
            builder: (BuildContext context) {
              return Directionality(
                textDirection: TextDirection.ltr,
                child: Align(
                  alignment: Alignment.topLeft,
                  child: MenuAnchor(
                    menuChildren: const <Widget>[
                      SubmenuButton(
                        alignmentOffset: Offset(10, 0),
                        menuChildren: <Widget>[
                          SubmenuButton(
                            menuChildren: <Widget>[
                              SubmenuButton(
                                alignmentOffset: Offset(10, 0),
                                menuChildren: <Widget>[
                                  SubmenuButton(
                                    menuChildren: <Widget>[],
                                    child: Text('SubMenuButton4'),
                                  ),
                                ],
                                child: Text('SubMenuButton3'),
                              ),
                            ],
                            child: Text('SubMenuButton2'),
                          ),
                        ],
                        child: Text('SubMenuButton1'),
                      ),
                    ],
                    builder: (BuildContext context, MenuController controller, Widget? child) {
                      return FilledButton(
                        onPressed: () {
                          if (controller.isOpen) {
                            controller.close();
                          } else {
                            controller.open();
                          }
                        },
                        child: const Text('Tap me'),
                      );
                    },
                  ),
                ),
              );
            },
          ),
        ),
      );
      await tester.pump();

      await tester.tap(find.text('Tap me'));
      await tester.pump();
      await tester.tap(find.text('SubMenuButton1'));
      await tester.pump();
      await tester.tap(find.text('SubMenuButton2'));
      await tester.pump();
      await tester.tap(find.text('SubMenuButton3'));
      await tester.pump();

      expect(find.byType(SubmenuButton), findsNWidgets(4));
      expect(
        collectSubmenuRects(),
        equals(const <Rect>[
          Rect.fromLTRB(0.0, 48.0, 256.0, 112.0),
          Rect.fromLTRB(266.0, 48.0, 522.0, 112.0),
          Rect.fromLTRB(522.0, 48.0, 778.0, 112.0),
          Rect.fromLTRB(256.0, 48.0, 512.0, 112.0),
        ]),
      );
    });

    testWidgets('constrained menus show up in the right place with offset in RTL', (
      WidgetTester tester,
    ) async {
      await changeSurfaceSize(tester, const Size(800, 600));
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Builder(
            builder: (BuildContext context) {
              return Directionality(
                textDirection: TextDirection.rtl,
                child: Align(
                  alignment: Alignment.topRight,
                  child: MenuAnchor(
                    menuChildren: const <Widget>[
                      SubmenuButton(
                        alignmentOffset: Offset(10, 0),
                        menuChildren: <Widget>[
                          SubmenuButton(
                            menuChildren: <Widget>[
                              SubmenuButton(
                                alignmentOffset: Offset(10, 0),
                                menuChildren: <Widget>[
                                  SubmenuButton(
                                    menuChildren: <Widget>[],
                                    child: Text('SubMenuButton4'),
                                  ),
                                ],
                                child: Text('SubMenuButton3'),
                              ),
                            ],
                            child: Text('SubMenuButton2'),
                          ),
                        ],
                        child: Text('SubMenuButton1'),
                      ),
                    ],
                    builder: (BuildContext context, MenuController controller, Widget? child) {
                      return FilledButton(
                        onPressed: () {
                          if (controller.isOpen) {
                            controller.close();
                          } else {
                            controller.open();
                          }
                        },
                        child: const Text('Tap me'),
                      );
                    },
                  ),
                ),
              );
            },
          ),
        ),
      );
      await tester.pump();

      await tester.tap(find.text('Tap me'));
      await tester.pump();
      await tester.tap(find.text('SubMenuButton1'));
      await tester.pump();
      await tester.tap(find.text('SubMenuButton2'));
      await tester.pump();
      await tester.tap(find.text('SubMenuButton3'));
      await tester.pump();

      expect(find.byType(SubmenuButton), findsNWidgets(4));
      expect(
        collectSubmenuRects(),
        equals(const <Rect>[
          Rect.fromLTRB(544.0, 48.0, 800.0, 112.0),
          Rect.fromLTRB(278.0, 48.0, 534.0, 112.0),
          Rect.fromLTRB(22.0, 48.0, 278.0, 112.0),
          Rect.fromLTRB(288.0, 48.0, 544.0, 112.0),
        ]),
      );
    });

    testWidgets('vertically constrained menus are positioned above the anchor by default', (
      WidgetTester tester,
    ) async {
      await changeSurfaceSize(tester, const Size(800, 600));
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Builder(
            builder: (BuildContext context) {
              return Directionality(
                textDirection: TextDirection.ltr,
                child: Align(
                  alignment: Alignment.bottomLeft,
                  child: MenuAnchor(
                    menuChildren: const <Widget>[MenuItemButton(child: Text('Button1'))],
                    builder: (BuildContext context, MenuController controller, Widget? child) {
                      return FilledButton(
                        onPressed: () {
                          if (controller.isOpen) {
                            controller.close();
                          } else {
                            controller.open();
                          }
                        },
                        child: const Text('Tap me'),
                      );
                    },
                  ),
                ),
              );
            },
          ),
        ),
      );

      await tester.pump();
      await tester.tap(find.text('Tap me'));
      await tester.pump();

      expect(find.byType(MenuItemButton), findsNWidgets(1));
      // Test the default offset (0, 0) vertical position.
      expect(collectSubmenuRects(), equals(const <Rect>[Rect.fromLTRB(0.0, 488.0, 122.0, 552.0)]));
    });

    testWidgets(
      'vertically constrained menus are positioned above the anchor with the provided offset',
      (WidgetTester tester) async {
        await changeSurfaceSize(tester, const Size(800, 600));
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(useMaterial3: false),
            home: Builder(
              builder: (BuildContext context) {
                return Directionality(
                  textDirection: TextDirection.ltr,
                  child: Align(
                    alignment: Alignment.bottomLeft,
                    child: MenuAnchor(
                      alignmentOffset: const Offset(0, 50),
                      menuChildren: const <Widget>[MenuItemButton(child: Text('Button1'))],
                      builder: (BuildContext context, MenuController controller, Widget? child) {
                        return FilledButton(
                          onPressed: () {
                            if (controller.isOpen) {
                              controller.close();
                            } else {
                              controller.open();
                            }
                          },
                          child: const Text('Tap me'),
                        );
                      },
                    ),
                  ),
                );
              },
            ),
          ),
        );

        await tester.pump();
        await tester.tap(find.text('Tap me'));
        await tester.pump();

        expect(find.byType(MenuItemButton), findsNWidgets(1));
        // Test the offset (0, 50) vertical position.
        expect(
          collectSubmenuRects(),
          equals(const <Rect>[Rect.fromLTRB(0.0, 438.0, 122.0, 502.0)]),
        );
      },
    );

    Future<void> buildDensityPaddingApp(
      WidgetTester tester, {
      required TextDirection textDirection,
      VisualDensity visualDensity = VisualDensity.standard,
      EdgeInsetsGeometry? menuPadding,
    }) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData.light(useMaterial3: false).copyWith(visualDensity: visualDensity),
          home: Directionality(
            textDirection: textDirection,
            child: Material(
              child: Column(
                children: <Widget>[
                  MenuBar(
                    style:
                        menuPadding != null
                            ? MenuStyle(
                              padding: MaterialStatePropertyAll<EdgeInsetsGeometry>(menuPadding),
                            )
                            : null,
                    children: createTestMenus(onPressed: onPressed),
                  ),
                  const Expanded(child: Placeholder()),
                ],
              ),
            ),
          ),
        ),
      );
      await tester.pump();
      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();
      await tester.tap(find.text(TestMenu.subMenu11.label));
      await tester.pump();
    }

    testWidgets('submenus account for density in LTR', (WidgetTester tester) async {
      await buildDensityPaddingApp(tester, textDirection: TextDirection.ltr);
      expect(
        collectSubmenuRects(),
        equals(const <Rect>[
          Rect.fromLTRB(145.0, 0.0, 655.0, 48.0),
          Rect.fromLTRB(257.0, 48.0, 471.0, 208.0),
          Rect.fromLTRB(471.0, 96.0, 719.0, 304.0),
        ]),
      );
    });

    testWidgets('submenus account for menu density in RTL', (WidgetTester tester) async {
      await buildDensityPaddingApp(tester, textDirection: TextDirection.rtl);
      expect(
        collectSubmenuRects(),
        equals(const <Rect>[
          Rect.fromLTRB(145.0, 0.0, 655.0, 48.0),
          Rect.fromLTRB(329.0, 48.0, 543.0, 208.0),
          Rect.fromLTRB(81.0, 96.0, 329.0, 304.0),
        ]),
      );
    });

    testWidgets('submenus account for compact menu density in LTR', (WidgetTester tester) async {
      await buildDensityPaddingApp(
        tester,
        visualDensity: VisualDensity.compact,
        textDirection: TextDirection.ltr,
      );
      expect(
        collectSubmenuRects(),
        equals(const <Rect>[
          Rect.fromLTRB(161.0, 0.0, 639.0, 40.0),
          Rect.fromLTRB(265.0, 40.0, 467.0, 160.0),
          Rect.fromLTRB(467.0, 80.0, 707.0, 240.0),
        ]),
      );
    });

    testWidgets('submenus account for compact menu density in RTL', (WidgetTester tester) async {
      await buildDensityPaddingApp(
        tester,
        visualDensity: VisualDensity.compact,
        textDirection: TextDirection.rtl,
      );
      expect(
        collectSubmenuRects(),
        equals(const <Rect>[
          Rect.fromLTRB(161.0, 0.0, 639.0, 40.0),
          Rect.fromLTRB(333.0, 40.0, 535.0, 160.0),
          Rect.fromLTRB(93.0, 80.0, 333.0, 240.0),
        ]),
      );
    });

    testWidgets('submenus account for padding in LTR', (WidgetTester tester) async {
      await buildDensityPaddingApp(
        tester,
        menuPadding: const EdgeInsetsDirectional.only(start: 10, end: 11, top: 12, bottom: 13),
        textDirection: TextDirection.ltr,
      );
      expect(
        collectSubmenuRects(),
        equals(const <Rect>[
          Rect.fromLTRB(138.5, 0.0, 661.5, 73.0),
          Rect.fromLTRB(256.5, 60.0, 470.5, 220.0),
          Rect.fromLTRB(470.5, 108.0, 718.5, 316.0),
        ]),
      );
    });

    testWidgets('submenus account for padding in RTL', (WidgetTester tester) async {
      await buildDensityPaddingApp(
        tester,
        menuPadding: const EdgeInsetsDirectional.only(start: 10, end: 11, top: 12, bottom: 13),
        textDirection: TextDirection.rtl,
      );
      expect(
        collectSubmenuRects(),
        equals(const <Rect>[
          Rect.fromLTRB(138.5, 0.0, 661.5, 73.0),
          Rect.fromLTRB(329.5, 60.0, 543.5, 220.0),
          Rect.fromLTRB(81.5, 108.0, 329.5, 316.0),
        ]),
      );
    });

    testWidgets('Menu follows content position when a LayerLink is provided', (
      WidgetTester tester,
    ) async {
      final MenuController controller = MenuController();
      final UniqueKey contentKey = UniqueKey();

      Widget boilerplate(double bottomInsets) {
        return MaterialApp(
          home: MediaQuery(
            data: MediaQueryData(viewInsets: EdgeInsets.only(bottom: bottomInsets)),
            child: Scaffold(
              body: Center(
                child: MenuAnchor(
                  controller: controller,
                  layerLink: LayerLink(),
                  menuChildren: <Widget>[
                    MenuItemButton(onPressed: () {}, child: const Text('Button 1')),
                  ],
                  builder: (BuildContext context, MenuController controller, Widget? child) {
                    return SizedBox(key: contentKey, width: 100, height: 100);
                  },
                ),
              ),
            ),
          ),
        );
      }

      // Build once without bottom insets and open the menu.
      await tester.pumpWidget(boilerplate(0.0));
      controller.open();
      await tester.pump();

      // Menu vertical position is just under the content.
      expect(tester.getRect(findMenuPanels()).top, tester.getRect(find.byKey(contentKey)).bottom);

      // Simulate the keyboard opening resizing the view.
      await tester.pumpWidget(boilerplate(100.0));
      await tester.pump();

      // Menu vertical position is just under the content.
      expect(tester.getRect(findMenuPanels()).top, tester.getRect(find.byKey(contentKey)).bottom);
    });

    testWidgets(
      'Menu is correctly offset when a LayerLink is provided and alignmentOffset is set',
      (WidgetTester tester) async {
        final MenuController controller = MenuController();
        final UniqueKey contentKey = UniqueKey();
        const double horizontalOffset = 16.0;
        const double verticalOffset = 20.0;

        await tester.pumpWidget(
          MaterialApp(
            home: Scaffold(
              body: Center(
                child: MenuAnchor(
                  controller: controller,
                  layerLink: LayerLink(),
                  alignmentOffset: const Offset(horizontalOffset, verticalOffset),
                  menuChildren: <Widget>[
                    MenuItemButton(onPressed: () {}, child: const Text('Button 1')),
                  ],
                  builder: (BuildContext context, MenuController controller, Widget? child) {
                    return SizedBox(key: contentKey, width: 100, height: 100);
                  },
                ),
              ),
            ),
          ),
        );

        controller.open();
        await tester.pump();

        expect(
          tester.getRect(findMenuPanels()).top,
          tester.getRect(find.byKey(contentKey)).bottom + verticalOffset,
        );
        expect(
          tester.getRect(findMenuPanels()).left,
          tester.getRect(find.byKey(contentKey)).left + horizontalOffset,
        );
      },
    );

    group('LocalizedShortcutLabeler', () {
      testWidgets('getShortcutLabel returns the right labels', (WidgetTester tester) async {
        String expectedMeta;
        String expectedCtrl;
        String expectedAlt;
        String expectedSeparator;
        String expectedShift;
        switch (defaultTargetPlatform) {
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
          case TargetPlatform.linux:
          case TargetPlatform.windows:
            expectedCtrl = 'Ctrl';
            expectedMeta = defaultTargetPlatform == TargetPlatform.windows ? 'Win' : 'Meta';
            expectedAlt = 'Alt';
            expectedShift = 'Shift';
            expectedSeparator = '+';
          case TargetPlatform.iOS:
          case TargetPlatform.macOS:
            expectedCtrl = '⌃';
            expectedMeta = '⌘';
            expectedAlt = '⌥';
            expectedShift = '⇧';
            expectedSeparator = ' ';
        }

        const SingleActivator allModifiers = SingleActivator(
          LogicalKeyboardKey.keyA,
          control: true,
          meta: true,
          shift: true,
          alt: true,
        );
        late String allExpected;
        switch (defaultTargetPlatform) {
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
          case TargetPlatform.linux:
          case TargetPlatform.windows:
            allExpected = <String>[
              expectedAlt,
              expectedCtrl,
              expectedMeta,
              expectedShift,
              'A',
            ].join(expectedSeparator);
          case TargetPlatform.iOS:
          case TargetPlatform.macOS:
            allExpected = <String>[
              expectedCtrl,
              expectedAlt,
              expectedShift,
              expectedMeta,
              'A',
            ].join(expectedSeparator);
        }
        const CharacterActivator charShortcuts = CharacterActivator('ñ');
        const String charExpected = 'ñ';
        await tester.pumpWidget(
          MaterialApp(
            home: Material(
              child: MenuBar(
                controller: controller,
                children: <Widget>[
                  SubmenuButton(
                    menuChildren: <Widget>[
                      MenuItemButton(shortcut: allModifiers, child: Text(TestMenu.subMenu10.label)),
                      MenuItemButton(
                        shortcut: charShortcuts,
                        child: Text(TestMenu.subMenu11.label),
                      ),
                    ],
                    child: Text(TestMenu.mainMenu0.label),
                  ),
                ],
              ),
            ),
          ),
        );
        await tester.tap(find.text(TestMenu.mainMenu0.label));
        await tester.pump();

        expect(find.text(allExpected), findsOneWidget);
        expect(find.text(charExpected), findsOneWidget);
      }, variant: TargetPlatformVariant.all());
    });

    group('CheckboxMenuButton', () {
      testWidgets('tapping toggles checkbox', (WidgetTester tester) async {
        bool? checkBoxValue;
        await tester.pumpWidget(
          MaterialApp(
            home: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return MenuBar(
                  children: <Widget>[
                    SubmenuButton(
                      menuChildren: <Widget>[
                        CheckboxMenuButton(
                          value: checkBoxValue,
                          onChanged: (bool? value) {
                            setState(() {
                              checkBoxValue = value;
                            });
                          },
                          tristate: true,
                          child: const Text('checkbox'),
                        ),
                      ],
                      child: const Text('submenu'),
                    ),
                  ],
                );
              },
            ),
          ),
        );

        await tester.tap(find.byType(SubmenuButton));
        await tester.pump();

        expect(tester.widget<CheckboxMenuButton>(find.byType(CheckboxMenuButton)).value, null);

        await tester.tap(find.byType(CheckboxMenuButton));
        await tester.pumpAndSettle();
        expect(checkBoxValue, false);

        await tester.tap(find.byType(SubmenuButton));
        await tester.pump();
        await tester.tap(find.byType(CheckboxMenuButton));
        await tester.pumpAndSettle();
        expect(checkBoxValue, true);

        await tester.tap(find.byType(SubmenuButton));
        await tester.pump();
        await tester.tap(find.byType(CheckboxMenuButton));
        await tester.pumpAndSettle();
        expect(checkBoxValue, null);
      });
    });

    group('RadioMenuButton', () {
      testWidgets('tapping toggles radio button', (WidgetTester tester) async {
        int? radioValue;
        await tester.pumpWidget(
          MaterialApp(
            home: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return MenuBar(
                  children: <Widget>[
                    SubmenuButton(
                      menuChildren: <Widget>[
                        RadioMenuButton<int>(
                          value: 0,
                          groupValue: radioValue,
                          onChanged: (int? value) {
                            setState(() {
                              radioValue = value;
                            });
                          },
                          toggleable: true,
                          child: const Text('radio 0'),
                        ),
                        RadioMenuButton<int>(
                          value: 1,
                          groupValue: radioValue,
                          onChanged: (int? value) {
                            setState(() {
                              radioValue = value;
                            });
                          },
                          toggleable: true,
                          child: const Text('radio 1'),
                        ),
                      ],
                      child: const Text('submenu'),
                    ),
                  ],
                );
              },
            ),
          ),
        );

        await tester.tap(find.byType(SubmenuButton));
        await tester.pump();

        expect(
          tester.widget<RadioMenuButton<int>>(find.byType(RadioMenuButton<int>).first).groupValue,
          null,
        );

        await tester.tap(find.byType(RadioMenuButton<int>).first);
        await tester.pumpAndSettle();
        expect(radioValue, 0);

        await tester.tap(find.byType(SubmenuButton));
        await tester.pump();
        await tester.tap(find.byType(RadioMenuButton<int>).first);
        await tester.pumpAndSettle();
        expect(radioValue, null);

        await tester.tap(find.byType(SubmenuButton));
        await tester.pump();
        await tester.tap(find.byType(RadioMenuButton<int>).last);
        await tester.pumpAndSettle();
        expect(radioValue, 1);
      });
    });

    group('Semantics', () {
      testWidgets('MenuItemButton is not a semantic button', (WidgetTester tester) async {
        final SemanticsTester semantics = SemanticsTester(tester);
        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: Center(
              child: MenuItemButton(
                style: MenuItemButton.styleFrom(fixedSize: const Size(88.0, 36.0)),
                onPressed: () {},
                child: const Text('ABC'),
              ),
            ),
          ),
        );

        // The flags should not have SemanticsFlag.isButton.
        expect(
          semantics,
          hasSemantics(
            TestSemantics.root(
              children: <TestSemantics>[
                TestSemantics.rootChild(
                  actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
                  label: 'ABC',
                  rect: const Rect.fromLTRB(0.0, 0.0, 88.0, 48.0),
                  transform: Matrix4.translationValues(356.0, 276.0, 0.0),
                  flags: <SemanticsFlag>[
                    SemanticsFlag.hasEnabledState,
                    SemanticsFlag.isEnabled,
                    SemanticsFlag.isFocusable,
                  ],
                  textDirection: TextDirection.ltr,
                ),
              ],
            ),
            ignoreId: true,
          ),
        );

        semantics.dispose();
      });

      testWidgets('MenuItemButton semantics respects label', (WidgetTester tester) async {
        final SemanticsTester semantics = SemanticsTester(tester);
        await tester.pumpWidget(
          MaterialApp(
            home: Center(
              child: MenuItemButton(
                semanticsLabel: 'TestWidget',
                shortcut: const SingleActivator(LogicalKeyboardKey.comma),
                style: MenuItemButton.styleFrom(fixedSize: const Size(88.0, 36.0)),
                onPressed: () {},
                child: const Text('ABC'),
              ),
            ),
          ),
        );

        expect(find.bySemanticsLabel('TestWidget'), findsOneWidget);
        semantics.dispose();
      }, variant: TargetPlatformVariant.desktop());

      testWidgets('SubMenuButton is not a semantic button', (WidgetTester tester) async {
        final SemanticsTester semantics = SemanticsTester(tester);
        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: Center(
              child: SubmenuButton(
                onHover: (bool value) {},
                style: SubmenuButton.styleFrom(fixedSize: const Size(88.0, 36.0)),
                menuChildren: const <Widget>[],
                child: const Text('ABC'),
              ),
            ),
          ),
        );

        // The flags should not have SemanticsFlag.isButton.
        expect(
          semantics,
          hasSemantics(
            TestSemantics.root(
              children: <TestSemantics>[
                TestSemantics(
                  rect: const Rect.fromLTRB(0.0, 0.0, 88.0, 48.0),
                  flags: <SemanticsFlag>[
                    SemanticsFlag.hasEnabledState,
                    SemanticsFlag.hasExpandedState,
                  ],
                  label: 'ABC',
                  textDirection: TextDirection.ltr,
                ),
              ],
            ),
            ignoreTransform: true,
            ignoreId: true,
          ),
        );

        semantics.dispose();
      });

      testWidgets('SubmenuButton expanded/collapsed state', (WidgetTester tester) async {
        final SemanticsTester semantics = SemanticsTester(tester);
        await tester.pumpWidget(
          MaterialApp(
            home: Center(
              child: SubmenuButton(
                style: SubmenuButton.styleFrom(fixedSize: const Size(88.0, 36.0)),
                menuChildren: <Widget>[
                  MenuItemButton(
                    style: MenuItemButton.styleFrom(fixedSize: const Size(120.0, 36.0)),
                    child: const Text('Item 0'),
                    onPressed: () {},
                  ),
                ],
                child: const Text('ABC'),
              ),
            ),
          ),
        );

        // Test expanded state.
        await tester.tap(find.text('ABC'));
        await tester.pumpAndSettle();
        expect(
          semantics,
          hasSemantics(
            TestSemantics.root(
              children: <TestSemantics>[
                TestSemantics(
                  id: 1,
                  rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 600.0),
                  children: <TestSemantics>[
                    TestSemantics(
                      id: 2,
                      rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 600.0),
                      children: <TestSemantics>[
                        TestSemantics(
                          id: 3,
                          rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 600.0),
                          flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                          children: <TestSemantics>[
                            TestSemantics(
                              id: 4,
                              flags: <SemanticsFlag>[
                                SemanticsFlag.isFocused,
                                SemanticsFlag.hasEnabledState,
                                SemanticsFlag.isEnabled,
                                SemanticsFlag.isFocusable,
                                SemanticsFlag.hasExpandedState,
                                SemanticsFlag.isExpanded,
                              ],
                              actions: <SemanticsAction>[
                                SemanticsAction.tap,
                                SemanticsAction.focus,
                              ],
                              label: 'ABC',
                              rect: const Rect.fromLTRB(0.0, 0.0, 88.0, 48.0),
                            ),
                            TestSemantics(
                              id: 6,
                              rect: const Rect.fromLTRB(0.0, 0.0, 120.0, 64.0),
                              children: <TestSemantics>[
                                TestSemantics(
                                  id: 7,
                                  rect: const Rect.fromLTRB(0.0, 0.0, 120.0, 48.0),
                                  flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                                  children: <TestSemantics>[
                                    TestSemantics(
                                      id: 8,
                                      label: 'Item 0',
                                      rect: const Rect.fromLTRB(0.0, 0.0, 120.0, 48.0),
                                      flags: <SemanticsFlag>[
                                        SemanticsFlag.hasEnabledState,
                                        SemanticsFlag.isEnabled,
                                        SemanticsFlag.isFocusable,
                                      ],
                                      actions: <SemanticsAction>[
                                        SemanticsAction.tap,
                                        SemanticsAction.focus,
                                      ],
                                    ),
                                  ],
                                ),
                              ],
                            ),
                          ],
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
            ignoreTransform: true,
          ),
        );

        // Test collapsed state.
        await tester.tap(find.text('ABC'));
        await tester.pumpAndSettle();
        expect(
          semantics,
          hasSemantics(
            TestSemantics.root(
              children: <TestSemantics>[
                TestSemantics(
                  id: 1,
                  rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 600.0),
                  children: <TestSemantics>[
                    TestSemantics(
                      id: 2,
                      rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 600.0),
                      children: <TestSemantics>[
                        TestSemantics(
                          id: 3,
                          rect: const Rect.fromLTRB(0.0, 0.0, 800.0, 600.0),
                          flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                          children: <TestSemantics>[
                            TestSemantics(
                              id: 4,
                              flags: <SemanticsFlag>[
                                SemanticsFlag.hasExpandedState,
                                SemanticsFlag.isFocused,
                                SemanticsFlag.hasEnabledState,
                                SemanticsFlag.isEnabled,
                                SemanticsFlag.isFocusable,
                              ],
                              actions: <SemanticsAction>[
                                SemanticsAction.tap,
                                SemanticsAction.focus,
                              ],
                              label: 'ABC',
                              rect: const Rect.fromLTRB(0.0, 0.0, 88.0, 48.0),
                            ),
                          ],
                        ),
                      ],
                    ),
                  ],
                ),
              ],
            ),
            ignoreTransform: true,
          ),
        );

        semantics.dispose();
      });
    });

    // This is a regression test for https://github.com/flutter/flutter/issues/131676.
    testWidgets('Material3 - Menu uses correct text styles', (WidgetTester tester) async {
      const TextStyle menuTextStyle = TextStyle(
        fontSize: 18.5,
        fontStyle: FontStyle.italic,
        wordSpacing: 1.2,
        decoration: TextDecoration.lineThrough,
      );
      final ThemeData themeData = ThemeData(textTheme: const TextTheme(labelLarge: menuTextStyle));
      await tester.pumpWidget(
        MaterialApp(
          theme: themeData,
          home: Material(
            child: MenuBar(
              controller: controller,
              children: createTestMenus(onPressed: onPressed, onOpen: onOpen, onClose: onClose),
            ),
          ),
        ),
      );

      // Test menu button text style uses the TextTheme.labelLarge.
      Finder buttonMaterial =
          find.descendant(of: find.byType(TextButton), matching: find.byType(Material)).first;
      Material material = tester.widget<Material>(buttonMaterial);
      expect(material.textStyle?.fontSize, menuTextStyle.fontSize);
      expect(material.textStyle?.fontStyle, menuTextStyle.fontStyle);
      expect(material.textStyle?.wordSpacing, menuTextStyle.wordSpacing);
      expect(material.textStyle?.decoration, menuTextStyle.decoration);

      // Open the menu.
      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();

      // Test menu item text style uses the TextTheme.labelLarge.
      buttonMaterial =
          find
              .descendant(
                of: find.widgetWithText(TextButton, TestMenu.subMenu10.label),
                matching: find.byType(Material),
              )
              .first;
      material = tester.widget<Material>(buttonMaterial);
      expect(material.textStyle?.fontSize, menuTextStyle.fontSize);
      expect(material.textStyle?.fontStyle, menuTextStyle.fontStyle);
      expect(material.textStyle?.wordSpacing, menuTextStyle.wordSpacing);
      expect(material.textStyle?.decoration, menuTextStyle.decoration);
    });

    testWidgets('SubmenuButton.onFocusChange is respected', (WidgetTester tester) async {
      final FocusNode focusNode = FocusNode();
      addTearDown(focusNode.dispose);
      int onFocusChangeCalled = 0;
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return SubmenuButton(
                  focusNode: focusNode,
                  onFocusChange: (bool value) {
                    setState(() {
                      onFocusChangeCalled += 1;
                    });
                  },
                  menuChildren: const <Widget>[MenuItemButton(child: Text('item 0'))],
                  child: const Text('Submenu 0'),
                );
              },
            ),
          ),
        ),
      );

      focusNode.requestFocus();
      await tester.pump();
      expect(focusNode.hasFocus, true);
      expect(onFocusChangeCalled, 1);

      focusNode.unfocus();
      await tester.pump();
      expect(focusNode.hasFocus, false);
      expect(onFocusChangeCalled, 2);
    });

    testWidgets('Horizontal _MenuPanel wraps children with IntrinsicWidth', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: MenuBar(
              children: <Widget>[MenuItemButton(onPressed: () {}, child: const Text('Menu Item'))],
            ),
          ),
        ),
      );

      // Horizontal _MenuPanel wraps children with IntrinsicWidth to ensure MenuItemButton
      // with vertical overflow axis is as wide as the widest child.
      final Finder intrinsicWidthFinder = find.ancestor(
        of: find.byType(MenuItemButton),
        matching: find.byType(IntrinsicWidth),
      );
      expect(intrinsicWidthFinder, findsOneWidget);
    });

    testWidgets('SubmenuButton.styleFrom overlayColor overrides default overlay color', (
      WidgetTester tester,
    ) async {
      const Color overlayColor = Color(0xffff00ff);
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: SubmenuButton(
              style: SubmenuButton.styleFrom(overlayColor: overlayColor),
              menuChildren: <Widget>[
                MenuItemButton(onPressed: () {}, child: const Text('MenuItemButton')),
              ],
              child: const Text('Submenu'),
            ),
          ),
        ),
      );

      // Hovered.
      final Offset center = tester.getCenter(find.byType(SubmenuButton));
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
          ..rect(color: overlayColor.withOpacity(0.08))
          ..rect(color: overlayColor.withOpacity(0.1)),
      );
    });

    testWidgets(
      'Garbage collector destroys child _MenuAnchorState after parent is closed',
      (WidgetTester tester) async {
        // Regression test for https://github.com/flutter/flutter/issues/149584
        await tester.pumpWidget(
          MaterialApp(
            home: MenuAnchor(
              controller: controller,
              menuChildren: const <Widget>[
                SubmenuButton(menuChildren: <Widget>[], child: Text('')),
              ],
            ),
          ),
        );

        controller.open();
        await tester.pump();

        final WeakReference<State> state = WeakReference<State>(
          tester.firstState<State<SubmenuButton>>(find.byType(SubmenuButton)),
        );
        expect(state.target, isNotNull);

        controller.close();
        await tester.pump();

        controller.open();
        await tester.pump();

        controller.close();
        await tester.pump();

        // Garbage collect. 1 should be enough, but 3 prevents flaky tests.
        await tester.runAsync<void>(() async {
          await forceGC(fullGcCycles: 3);
        });

        expect(state.target, isNull);
      },
      // Skipped on Web: [intended] ForceGC does not work in web and in release mode. See https://api.flutter.dev/flutter/package-leak_tracker_leak_tracker/forceGC.html
      // Skipped for everyone else: forceGC is flaky, see https://github.com/flutter/flutter/issues/154858
      skip: true,
    );

    // Regression test for https://github.com/flutter/flutter/issues/154798.
    testWidgets('MenuItemButton.styleFrom can customize the button icon', (
      WidgetTester tester,
    ) async {
      const Color iconColor = Color(0xFFF000FF);
      const double iconSize = 32.0;
      const Color disabledIconColor = Color(0xFFFFF000);
      Widget buildButton({bool enabled = true}) {
        return MaterialApp(
          home: Material(
            child: Center(
              child: MenuItemButton(
                style: MenuItemButton.styleFrom(
                  iconColor: iconColor,
                  iconSize: iconSize,
                  disabledIconColor: disabledIconColor,
                ),
                onPressed: enabled ? () {} : null,
                trailingIcon: const Icon(Icons.add),
                child: const Text('Button'),
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

    // Regression test for https://github.com/flutter/flutter/issues/154798.
    testWidgets('SubmenuButton.styleFrom can customize the button icon', (
      WidgetTester tester,
    ) async {
      const Color iconColor = Color(0xFFF000FF);
      const double iconSize = 32.0;
      const Color disabledIconColor = Color(0xFFFFF000);
      Widget buildButton({bool enabled = true}) {
        return MaterialApp(
          home: Material(
            child: Center(
              child: SubmenuButton(
                style: SubmenuButton.styleFrom(
                  iconColor: iconColor,
                  iconSize: iconSize,
                  disabledIconColor: disabledIconColor,
                ),
                trailingIcon: const Icon(Icons.add),
                menuChildren: <Widget>[if (enabled) const Text('Item')],
                child: const Text('SubmenuButton'),
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

    // Regression test for https://github.com/flutter/flutter/issues/155034.
    testWidgets('Content is shown in the root overlay when useRootOverlay is true', (
      WidgetTester tester,
    ) async {
      final MenuController controller = MenuController();
      final UniqueKey overlayKey = UniqueKey();
      final UniqueKey menuItemKey = UniqueKey();

      late final OverlayEntry overlayEntry;
      addTearDown(() {
        overlayEntry.remove();
        overlayEntry.dispose();
      });

      Widget boilerplate() {
        return MaterialApp(
          home: Overlay(
            key: overlayKey,
            initialEntries: <OverlayEntry>[
              overlayEntry = OverlayEntry(
                builder: (BuildContext context) {
                  return Scaffold(
                    body: Center(
                      child: MenuAnchor(
                        useRootOverlay: true,
                        controller: controller,
                        menuChildren: <Widget>[
                          MenuItemButton(
                            key: menuItemKey,
                            onPressed: () {},
                            child: const Text('Item 1'),
                          ),
                        ],
                      ),
                    ),
                  );
                },
              ),
            ],
          ),
        );
      }

      await tester.pumpWidget(boilerplate());
      expect(find.byKey(menuItemKey), findsNothing);

      // Open the menu.
      controller.open();
      await tester.pump();
      expect(find.byKey(menuItemKey), findsOne);

      // Expect two overlays: the root overlay created by MaterialApp and the
      // overlay created by the boilerplate code.
      expect(find.byType(Overlay), findsNWidgets(2));

      final Iterable<Overlay> overlays = tester.widgetList<Overlay>(find.byType(Overlay));
      final Overlay nonRootOverlay = tester.widget(find.byKey(overlayKey));
      final Overlay rootOverlay = overlays.firstWhere(
        (Overlay overlay) => overlay != nonRootOverlay,
      );

      // Check that the ancestor _RenderTheater for the menu item is the one
      // from the root overlay.
      expect(
        ancestorRenderTheaters(tester.renderObject(find.byKey(menuItemKey))).single,
        tester.renderObject(find.byWidget(rootOverlay)),
      );
    });

    testWidgets('Content is shown in the nearest ancestor overlay when useRootOverlay is false', (
      WidgetTester tester,
    ) async {
      final MenuController controller = MenuController();
      final UniqueKey overlayKey = UniqueKey();
      final UniqueKey menuItemKey = UniqueKey();

      late final OverlayEntry overlayEntry;
      addTearDown(() {
        overlayEntry.remove();
        overlayEntry.dispose();
      });

      Widget boilerplate() {
        return MaterialApp(
          home: Overlay(
            key: overlayKey,
            initialEntries: <OverlayEntry>[
              overlayEntry = OverlayEntry(
                builder: (BuildContext context) {
                  return Scaffold(
                    body: Center(
                      child: MenuAnchor(
                        controller: controller,
                        menuChildren: <Widget>[
                          MenuItemButton(
                            key: menuItemKey,
                            onPressed: () {},
                            child: const Text('Item 1'),
                          ),
                        ],
                      ),
                    ),
                  );
                },
              ),
            ],
          ),
        );
      }

      await tester.pumpWidget(boilerplate());
      expect(find.byKey(menuItemKey), findsNothing);

      // Open the menu.
      controller.open();
      await tester.pump();
      expect(find.byKey(menuItemKey), findsOne);

      // Expect two overlays: the root overlay created by MaterialApp and the
      // overlay created by the boilerplate code.
      expect(find.byType(Overlay), findsNWidgets(2));

      final Overlay nonRootOverlay = tester.widget(find.byKey(overlayKey));

      // Check that the ancestor _RenderTheater for the menu item is the one
      // from the root overlay.
      expect(
        ancestorRenderTheaters(tester.renderObject(find.byKey(menuItemKey))).first,
        tester.renderObject(find.byWidget(nonRootOverlay)),
      );
    });

    // Regression test for https://github.com/flutter/flutter/issues/156572.
    testWidgets('Unattached MenuController does not throw when calling close', (
      WidgetTester tester,
    ) async {
      final MenuController controller = MenuController();
      controller.close();
      await tester.pump();
      expect(tester.takeException(), isNull);
    });

    testWidgets('Unattached MenuController returns false when calling isOpen', (
      WidgetTester tester,
    ) async {
      final MenuController controller = MenuController();
      expect(controller.isOpen, false);
    });

    // Regression test for https://github.com/flutter/flutter/issues/157606.
    testWidgets('MenuAnchor updates isOpen state correctly', (WidgetTester tester) async {
      bool isOpen = false;
      int openCount = 0;
      int closeCount = 0;

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: MenuAnchor(
                menuChildren: const <Widget>[MenuItemButton(child: Text('menu item'))],
                builder: (BuildContext context, MenuController controller, Widget? child) {
                  isOpen = controller.isOpen;
                  return FilledButton(
                    onPressed: () {
                      if (controller.isOpen) {
                        controller.close();
                      } else {
                        controller.open();
                      }
                    },
                    child: Text(isOpen ? 'close' : 'open'),
                  );
                },
                onOpen: () => openCount++,
                onClose: () => closeCount++,
              ),
            ),
          ),
        ),
      );

      expect(find.text('open'), findsOneWidget);
      expect(isOpen, false);
      expect(openCount, 0);
      expect(closeCount, 0);

      await tester.tap(find.byType(FilledButton));
      await tester.pump();

      expect(find.text('close'), findsOneWidget);
      expect(isOpen, true);
      expect(openCount, 1);
      expect(closeCount, 0);

      await tester.tap(find.byType(FilledButton));
      await tester.pump();

      expect(find.text('open'), findsOneWidget);
      expect(isOpen, false);
      expect(openCount, 1);
      expect(closeCount, 1);
    });

    testWidgets('SubmenuButton.submenuIcon updates default arrow icon', (
      WidgetTester tester,
    ) async {
      const IconData disabledIcon = Icons.close;
      const IconData hoveredIcon = Icons.bolt;
      const IconData focusedIcon = Icons.favorite;
      const IconData defaultIcon = Icons.add;
      final WidgetStateProperty<Widget?> submenuIcon = WidgetStateProperty.resolveWith<Widget?>((
        Set<WidgetState> states,
      ) {
        if (states.contains(WidgetState.disabled)) {
          return const Icon(disabledIcon);
        }
        if (states.contains(WidgetState.hovered)) {
          return const Icon(hoveredIcon);
        }
        if (states.contains(WidgetState.focused)) {
          return const Icon(focusedIcon);
        }
        return const Icon(defaultIcon);
      });

      Widget buildMenu({WidgetStateProperty<Widget?>? icon, bool enabled = true}) {
        return MaterialApp(
          home: Material(
            child: MenuBar(
              controller: controller,
              children: <Widget>[
                SubmenuButton(
                  menuChildren: <Widget>[
                    SubmenuButton(
                      submenuIcon: icon,
                      menuChildren:
                          enabled
                              ? <Widget>[MenuItemButton(child: Text(TestMenu.mainMenu0.label))]
                              : <Widget>[],
                      child: Text(TestMenu.subSubMenu110.label),
                    ),
                  ],
                  child: Text(TestMenu.subMenu00.label),
                ),
              ],
            ),
          ),
        );
      }

      await tester.pumpWidget(buildMenu());
      await tester.tap(find.text(TestMenu.subMenu00.label));
      await tester.pump();

      expect(find.byIcon(Icons.arrow_right), findsOneWidget);

      controller.close();
      await tester.pump();

      await tester.pumpWidget(buildMenu(icon: submenuIcon));
      await tester.tap(find.text(TestMenu.subMenu00.label));
      await tester.pump();
      expect(find.byIcon(defaultIcon), findsOneWidget);

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.pump();
      expect(find.byIcon(focusedIcon), findsOneWidget);

      controller.close();
      await tester.pump();

      await tester.tap(find.text(TestMenu.subMenu00.label));
      await tester.pump();
      await hoverOver(tester, find.text(TestMenu.subSubMenu110.label));
      await tester.pump();
      expect(find.byIcon(hoveredIcon), findsOneWidget);

      controller.close();
      await tester.pump();

      await tester.pumpWidget(buildMenu(icon: submenuIcon, enabled: false));
      await tester.tap(find.text(TestMenu.subMenu00.label));
      await tester.pump();
      expect(find.byIcon(disabledIcon), findsOneWidget);
    });
  });
}

List<Widget> createTestMenus({
  void Function(TestMenu)? onPressed,
  void Function(TestMenu)? onOpen,
  void Function(TestMenu)? onClose,
  Map<TestMenu, MenuSerializableShortcut> shortcuts = const <TestMenu, MenuSerializableShortcut>{},
  bool includeExtraGroups = false,
  bool accelerators = false,
}) {
  Widget submenuButton(TestMenu menu, {required List<Widget> menuChildren}) {
    return SubmenuButton(
      onOpen: onOpen != null ? () => onOpen(menu) : null,
      onClose: onClose != null ? () => onClose(menu) : null,
      menuChildren: menuChildren,
      child: accelerators ? MenuAcceleratorLabel(menu.acceleratorLabel) : Text(menu.label),
    );
  }

  Widget menuItemButton(
    TestMenu menu, {
    bool enabled = true,
    Widget? leadingIcon,
    Widget? trailingIcon,
    Key? key,
  }) {
    return MenuItemButton(
      key: key,
      onPressed: enabled && onPressed != null ? () => onPressed(menu) : null,
      shortcut: shortcuts[menu],
      leadingIcon: leadingIcon,
      trailingIcon: trailingIcon,
      child: accelerators ? MenuAcceleratorLabel(menu.acceleratorLabel) : Text(menu.label),
    );
  }

  final List<Widget> result = <Widget>[
    submenuButton(
      TestMenu.mainMenu0,
      menuChildren: <Widget>[
        menuItemButton(TestMenu.subMenu00, leadingIcon: const Icon(Icons.add)),
        menuItemButton(TestMenu.subMenu01),
        menuItemButton(TestMenu.subMenu02),
      ],
    ),
    submenuButton(
      TestMenu.mainMenu1,
      menuChildren: <Widget>[
        menuItemButton(TestMenu.subMenu10),
        submenuButton(
          TestMenu.subMenu11,
          menuChildren: <Widget>[
            menuItemButton(TestMenu.subSubMenu110, key: UniqueKey()),
            menuItemButton(TestMenu.subSubMenu111),
            menuItemButton(TestMenu.subSubMenu112),
            menuItemButton(TestMenu.subSubMenu113),
          ],
        ),
        menuItemButton(TestMenu.subMenu12),
      ],
    ),
    submenuButton(
      TestMenu.mainMenu2,
      menuChildren: <Widget>[
        menuItemButton(TestMenu.subMenu20, leadingIcon: const Icon(Icons.ac_unit), enabled: false),
      ],
    ),
    if (includeExtraGroups)
      submenuButton(
        TestMenu.mainMenu3,
        menuChildren: <Widget>[menuItemButton(TestMenu.subMenu30, enabled: false)],
      ),
    if (includeExtraGroups)
      submenuButton(
        TestMenu.mainMenu4,
        menuChildren: <Widget>[
          menuItemButton(TestMenu.subMenu40, enabled: false),
          menuItemButton(TestMenu.subMenu41, enabled: false),
          menuItemButton(TestMenu.subMenu42, enabled: false),
        ],
      ),
    submenuButton(TestMenu.mainMenu5, menuChildren: const <Widget>[]),
  ];
  return result;
}

enum TestMenu {
  mainMenu0('&Menu 0'),
  mainMenu1('M&enu &1'),
  mainMenu2('Me&nu 2'),
  mainMenu3('Men&u 3'),
  mainMenu4('Menu &4'),
  mainMenu5('Menu &5 && &6 &'),
  subMenu00('Sub &Menu 0&0'),
  subMenu01('Sub Menu 0&1'),
  subMenu02('Sub Menu 0&2'),
  subMenu10('Sub Menu 1&0'),
  subMenu11('Sub Menu 1&1'),
  subMenu12('Sub Menu 1&2'),
  subMenu20('Sub Menu 2&0'),
  subMenu30('Sub Menu 3&0'),
  subMenu40('Sub Menu 4&0'),
  subMenu41('Sub Menu 4&1'),
  subMenu42('Sub Menu 4&2'),
  subSubMenu110('Sub Sub Menu 11&0'),
  subSubMenu111('Sub Sub Menu 11&1'),
  subSubMenu112('Sub Sub Menu 11&2'),
  subSubMenu113('Sub Sub Menu 11&3'),
  anchorButton('Press Me'),
  outsideButton('Outside');

  const TestMenu(this.acceleratorLabel);
  final String acceleratorLabel;
  // Strip the accelerator markers.
  String get label => MenuAcceleratorLabel.stripAcceleratorMarkers(acceleratorLabel);
}
