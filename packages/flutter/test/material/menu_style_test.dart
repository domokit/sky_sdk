// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  Finder findMenuPanels() {
    return find.byWidgetPredicate((Widget widget) => widget.runtimeType.toString() == '_MenuPanel');
  }

  Material getMenuBarMaterial(WidgetTester tester) {
    return tester.widget<Material>(
      find.descendant(of: findMenuPanels(), matching: find.byType(Material)).first,
    );
  }

  Padding getMenuBarPadding(WidgetTester tester) {
    return tester.widget<Padding>(
      find.descendant(of: findMenuPanels(), matching: find.byType(Padding)).first,
    );
  }

  Material getMenuMaterial(WidgetTester tester) {
    return tester.widget<Material>(
      find.descendant(of: findMenuPanels().at(1), matching: find.byType(Material)).first,
    );
  }

  Padding getMenuPadding(WidgetTester tester) {
    return tester.widget<Padding>(
      find.descendant(of: findMenuPanels().at(1), matching: find.byType(Padding)).first,
    );
  }

  group('MenuStyle', () {
    test('MenuStyle lerp special cases', () {
      expect(MenuStyle.lerp(null, null, 0), null);
      const MenuStyle data = MenuStyle();
      expect(identical(MenuStyle.lerp(data, data, 0.5), data), true);
    });

    testWidgets('fixedSize affects geometry', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Column(
              children: <Widget>[
                MenuBarTheme(
                  data: const MenuBarThemeData(
                    style: MenuStyle(fixedSize: MaterialStatePropertyAll<Size>(Size(600, 60))),
                  ),
                  child: MenuTheme(
                    data: const MenuThemeData(
                      style: MenuStyle(fixedSize: MaterialStatePropertyAll<Size>(Size(100, 100))),
                    ),
                    child: MenuBar(children: createTestMenus(onPressed: (TestMenu menu) {})),
                  ),
                ),
                const Expanded(child: Placeholder()),
              ],
            ),
          ),
        ),
      );

      // Have to open a menu initially to start things going.
      await tester.tap(find.text(TestMenu.mainMenu0.label));
      await tester.pump();

      // MenuBarTheme affects MenuBar.
      expect(
        tester.getRect(findMenuPanels().first),
        equals(const Rect.fromLTRB(100.0, 0.0, 700.0, 60.0)),
      );
      expect(tester.getRect(findMenuPanels().first).size, equals(const Size(600.0, 60.0)));

      // MenuTheme affects menus.
      expect(
        tester.getRect(findMenuPanels().at(1)),
        equals(const Rect.fromLTRB(104.0, 54.0, 204.0, 154.0)),
      );
      expect(tester.getRect(findMenuPanels().at(1)).size, equals(const Size(100.0, 100.0)));
    });

    testWidgets('maximumSize affects geometry', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Column(
              children: <Widget>[
                MenuBarTheme(
                  data: const MenuBarThemeData(
                    style: MenuStyle(maximumSize: MaterialStatePropertyAll<Size>(Size(250, 40))),
                  ),
                  child: MenuTheme(
                    data: const MenuThemeData(
                      style: MenuStyle(maximumSize: MaterialStatePropertyAll<Size>(Size(100, 100))),
                    ),
                    child: MenuBar(children: createTestMenus(onPressed: (TestMenu menu) {})),
                  ),
                ),
                const Expanded(child: Placeholder()),
              ],
            ),
          ),
        ),
      );

      // Have to open a menu initially to start things going.
      await tester.tap(find.text(TestMenu.mainMenu0.label));
      await tester.pump();

      // MenuBarTheme affects MenuBar.
      expect(
        tester.getRect(findMenuPanels().first),
        equals(const Rect.fromLTRB(275.0, 0.0, 525.0, 40.0)),
      );
      expect(tester.getRect(findMenuPanels().first).size, equals(const Size(250.0, 40.0)));

      // MenuTheme affects menus.
      expect(
        tester.getRect(findMenuPanels().at(1)),
        equals(const Rect.fromLTRB(279.0, 44.0, 379.0, 144.0)),
      );
      expect(tester.getRect(findMenuPanels().at(1)).size, equals(const Size(100.0, 100.0)));
    });

    testWidgets('minimumSize affects geometry', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Column(
              children: <Widget>[
                MenuBarTheme(
                  data: const MenuBarThemeData(
                    style: MenuStyle(minimumSize: MaterialStatePropertyAll<Size>(Size(400, 60))),
                  ),
                  child: MenuTheme(
                    data: const MenuThemeData(
                      style: MenuStyle(minimumSize: MaterialStatePropertyAll<Size>(Size(300, 300))),
                    ),
                    child: MenuBar(children: createTestMenus(onPressed: (TestMenu menu) {})),
                  ),
                ),
                const Expanded(child: Placeholder()),
              ],
            ),
          ),
        ),
      );

      // Have to open a menu initially to start things going.
      await tester.tap(find.text(TestMenu.mainMenu0.label));
      await tester.pump();

      // MenuBarTheme affects MenuBar.
      expect(
        tester.getRect(findMenuPanels().first),
        equals(const Rect.fromLTRB(200.0, 0.0, 600.0, 60.0)),
      );
      expect(tester.getRect(findMenuPanels().first).size, equals(const Size(400.0, 60.0)));

      // MenuTheme affects menus.
      expect(
        tester.getRect(findMenuPanels().at(1)),
        equals(const Rect.fromLTRB(204.0, 54.0, 504.0, 354.0)),
      );
      expect(tester.getRect(findMenuPanels().at(1)).size, equals(const Size(300.0, 300.0)));
    });

    testWidgets('Material parameters are honored', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Column(
              children: <Widget>[
                MenuBarTheme(
                  data: const MenuBarThemeData(
                    style: MenuStyle(
                      backgroundColor: MaterialStatePropertyAll<Color>(Colors.red),
                      shadowColor: MaterialStatePropertyAll<Color>(Colors.green),
                      surfaceTintColor: MaterialStatePropertyAll<Color>(Colors.blue),
                      padding: MaterialStatePropertyAll<EdgeInsetsGeometry>(EdgeInsets.all(10)),
                      elevation: MaterialStatePropertyAll<double>(10),
                      side: MaterialStatePropertyAll<BorderSide>(
                        BorderSide(color: Colors.redAccent),
                      ),
                      shape: MaterialStatePropertyAll<OutlinedBorder>(StadiumBorder()),
                    ),
                  ),
                  child: MenuTheme(
                    data: const MenuThemeData(
                      style: MenuStyle(
                        backgroundColor: MaterialStatePropertyAll<Color>(Colors.cyan),
                        shadowColor: MaterialStatePropertyAll<Color>(Colors.purple),
                        surfaceTintColor: MaterialStatePropertyAll<Color>(Colors.yellow),
                        padding: MaterialStatePropertyAll<EdgeInsetsGeometry>(EdgeInsets.all(20)),
                        elevation: MaterialStatePropertyAll<double>(20),
                        side: MaterialStatePropertyAll<BorderSide>(
                          BorderSide(color: Colors.cyanAccent),
                        ),
                        shape: MaterialStatePropertyAll<OutlinedBorder>(StarBorder()),
                      ),
                    ),
                    child: MenuBar(children: createTestMenus(onPressed: (TestMenu menu) {})),
                  ),
                ),
                const Expanded(child: Placeholder()),
              ],
            ),
          ),
        ),
      );

      // Have to open a menu initially to start things going.
      await tester.tap(find.text(TestMenu.mainMenu0.label));
      await tester.pump();

      final Material menuBarMaterial = getMenuBarMaterial(tester);
      final Padding menuBarPadding = getMenuBarPadding(tester);
      final Material panelMaterial = getMenuMaterial(tester);
      final Padding panelPadding = getMenuPadding(tester);

      // MenuBarTheme affects MenuBar.
      expect(menuBarMaterial.color, equals(Colors.red));
      expect(menuBarMaterial.shadowColor, equals(Colors.green));
      expect(menuBarMaterial.surfaceTintColor, equals(Colors.blue));
      expect(
        menuBarMaterial.shape,
        equals(const StadiumBorder(side: BorderSide(color: Colors.redAccent))),
      );
      expect(menuBarPadding.padding, equals(const EdgeInsets.all(10)));

      // MenuBarTheme affects menus.
      expect(panelMaterial.color, equals(Colors.cyan));
      expect(panelMaterial.shadowColor, equals(Colors.purple));
      expect(panelMaterial.surfaceTintColor, equals(Colors.yellow));
      expect(
        panelMaterial.shape,
        equals(const StarBorder(side: BorderSide(color: Colors.cyanAccent))),
      );
      expect(panelPadding.padding, equals(const EdgeInsets.all(20)));
    });

    testWidgets('visual density', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(
            child: Column(
              children: <Widget>[
                MenuBarTheme(
                  data: const MenuBarThemeData(
                    style: MenuStyle(visualDensity: VisualDensity(horizontal: 1.5, vertical: -1.5)),
                  ),
                  child: MenuTheme(
                    data: const MenuThemeData(
                      style: MenuStyle(
                        visualDensity: VisualDensity(horizontal: 0.5, vertical: -0.5),
                      ),
                    ),
                    child: MenuBar(children: createTestMenus(onPressed: (TestMenu menu) {})),
                  ),
                ),
                const Expanded(child: Placeholder()),
              ],
            ),
          ),
        ),
      );
      await tester.pump();

      expect(
        tester.getRect(find.byType(MenuBar)),
        equals(const Rect.fromLTRB(228.0, 0.0, 572.0, 48.0)),
      );

      // Open and make sure things are the right size.
      await tester.tap(find.text(TestMenu.mainMenu1.label));
      await tester.pump();

      expect(
        tester.getRect(find.byType(MenuBar)),
        equals(const Rect.fromLTRB(228.0, 0.0, 572.0, 48.0)),
      );
      expect(
        tester.getRect(find.text(TestMenu.subMenu10.label)),
        equals(const Rect.fromLTRB(372.0, 68.0, 565.0, 82.0)),
      );
      expect(
        tester.getRect(
          find
              .ancestor(of: find.text(TestMenu.subMenu10.label), matching: find.byType(Material))
              .at(1),
        ),
        equals(const Rect.fromLTRB(352.0, 48.0, 585.0, 186.0)),
      );
    });
  });
}

List<Widget> createTestMenus({
  void Function(TestMenu)? onPressed,
  void Function(TestMenu)? onOpen,
  void Function(TestMenu)? onClose,
  Map<TestMenu, MenuSerializableShortcut> shortcuts = const <TestMenu, MenuSerializableShortcut>{},
  bool includeStandard = false,
  bool includeExtraGroups = false,
}) {
  final List<Widget> result = <Widget>[
    SubmenuButton(
      onOpen: onOpen != null ? () => onOpen(TestMenu.mainMenu0) : null,
      onClose: onClose != null ? () => onClose(TestMenu.mainMenu0) : null,
      menuChildren: <Widget>[
        MenuItemButton(
          onPressed: onPressed != null ? () => onPressed(TestMenu.subMenu00) : null,
          shortcut: shortcuts[TestMenu.subMenu00],
          child: Text(TestMenu.subMenu00.label),
        ),
        MenuItemButton(
          onPressed: onPressed != null ? () => onPressed(TestMenu.subMenu01) : null,
          shortcut: shortcuts[TestMenu.subMenu01],
          child: Text(TestMenu.subMenu01.label),
        ),
        MenuItemButton(
          onPressed: onPressed != null ? () => onPressed(TestMenu.subMenu02) : null,
          shortcut: shortcuts[TestMenu.subMenu02],
          child: Text(TestMenu.subMenu02.label),
        ),
      ],
      child: Text(TestMenu.mainMenu0.label),
    ),
    SubmenuButton(
      onOpen: onOpen != null ? () => onOpen(TestMenu.mainMenu1) : null,
      onClose: onClose != null ? () => onClose(TestMenu.mainMenu1) : null,
      menuChildren: <Widget>[
        MenuItemButton(
          onPressed: onPressed != null ? () => onPressed(TestMenu.subMenu10) : null,
          shortcut: shortcuts[TestMenu.subMenu10],
          child: Text(TestMenu.subMenu10.label),
        ),
        SubmenuButton(
          onOpen: onOpen != null ? () => onOpen(TestMenu.subMenu11) : null,
          onClose: onClose != null ? () => onClose(TestMenu.subMenu11) : null,
          menuChildren: <Widget>[
            MenuItemButton(
              key: UniqueKey(),
              onPressed: onPressed != null ? () => onPressed(TestMenu.subSubMenu110) : null,
              shortcut: shortcuts[TestMenu.subSubMenu110],
              child: Text(TestMenu.subSubMenu110.label),
            ),
            MenuItemButton(
              onPressed: onPressed != null ? () => onPressed(TestMenu.subSubMenu111) : null,
              shortcut: shortcuts[TestMenu.subSubMenu111],
              child: Text(TestMenu.subSubMenu111.label),
            ),
            MenuItemButton(
              onPressed: onPressed != null ? () => onPressed(TestMenu.subSubMenu112) : null,
              shortcut: shortcuts[TestMenu.subSubMenu112],
              child: Text(TestMenu.subSubMenu112.label),
            ),
            MenuItemButton(
              onPressed: onPressed != null ? () => onPressed(TestMenu.subSubMenu113) : null,
              shortcut: shortcuts[TestMenu.subSubMenu113],
              child: Text(TestMenu.subSubMenu113.label),
            ),
          ],
          child: Text(TestMenu.subMenu11.label),
        ),
        MenuItemButton(
          onPressed: onPressed != null ? () => onPressed(TestMenu.subMenu12) : null,
          shortcut: shortcuts[TestMenu.subMenu12],
          child: Text(TestMenu.subMenu12.label),
        ),
      ],
      child: Text(TestMenu.mainMenu1.label),
    ),
    SubmenuButton(
      onOpen: onOpen != null ? () => onOpen(TestMenu.mainMenu2) : null,
      onClose: onClose != null ? () => onClose(TestMenu.mainMenu2) : null,
      menuChildren: <Widget>[
        MenuItemButton(
          // Always disabled.
          shortcut: shortcuts[TestMenu.subMenu20],
          child: Text(TestMenu.subMenu20.label),
        ),
      ],
      child: Text(TestMenu.mainMenu2.label),
    ),
    if (includeExtraGroups)
      SubmenuButton(
        onOpen: onOpen != null ? () => onOpen(TestMenu.mainMenu3) : null,
        onClose: onClose != null ? () => onClose(TestMenu.mainMenu3) : null,
        menuChildren: <Widget>[
          MenuItemButton(
            // Always disabled.
            shortcut: shortcuts[TestMenu.subMenu30],
            // Always disabled.
            child: Text(TestMenu.subMenu30.label),
          ),
        ],
        child: Text(TestMenu.mainMenu3.label),
      ),
    if (includeExtraGroups)
      SubmenuButton(
        onOpen: onOpen != null ? () => onOpen(TestMenu.mainMenu4) : null,
        onClose: onClose != null ? () => onClose(TestMenu.mainMenu4) : null,
        menuChildren: <Widget>[
          MenuItemButton(
            // Always disabled.
            shortcut: shortcuts[TestMenu.subMenu40],
            // Always disabled.
            child: Text(TestMenu.subMenu40.label),
          ),
          MenuItemButton(
            // Always disabled.
            shortcut: shortcuts[TestMenu.subMenu41],
            // Always disabled.
            child: Text(TestMenu.subMenu41.label),
          ),
          MenuItemButton(
            // Always disabled.
            shortcut: shortcuts[TestMenu.subMenu42],
            // Always disabled.
            child: Text(TestMenu.subMenu42.label),
          ),
        ],
        child: Text(TestMenu.mainMenu4.label),
      ),
  ];
  return result;
}

enum TestMenu {
  mainMenu0('Menu 0'),
  mainMenu1('Menu 1'),
  mainMenu2('Menu 2'),
  mainMenu3('Menu 3'),
  mainMenu4('Menu 4'),
  subMenu00('Sub Menu 00'),
  subMenu01('Sub Menu 01'),
  subMenu02('Sub Menu 02'),
  subMenu10('Sub Menu 10'),
  subMenu11('Sub Menu 11'),
  subMenu12('Sub Menu 12'),
  subMenu20('Sub Menu 20'),
  subMenu30('Sub Menu 30'),
  subMenu40('Sub Menu 40'),
  subMenu41('Sub Menu 41'),
  subMenu42('Sub Menu 42'),
  subSubMenu110('Sub Sub Menu 110'),
  subSubMenu111('Sub Sub Menu 111'),
  subSubMenu112('Sub Sub Menu 112'),
  subSubMenu113('Sub Sub Menu 113');

  const TestMenu(this.label);
  final String label;
}
