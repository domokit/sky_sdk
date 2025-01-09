// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  const String longText = 'one two three four five six seven eight nine ten eleven twelve';
  final List<DropdownMenuEntry<TestMenu>> menuChildren = <DropdownMenuEntry<TestMenu>>[];
  final List<DropdownMenuEntry<TestMenu>> menuChildrenWithIcons = <DropdownMenuEntry<TestMenu>>[];
  const double leadingIconToInputPadding = 4.0;

  for (final TestMenu value in TestMenu.values) {
    final DropdownMenuEntry<TestMenu> entry = DropdownMenuEntry<TestMenu>(
      value: value,
      label: value.label,
    );
    menuChildren.add(entry);
  }

  ValueKey<String> leadingIconKey(TestMenu menuEntry) =>
      ValueKey<String>('leading-${menuEntry.label}');
  ValueKey<String> trailingIconKey(TestMenu menuEntry) =>
      ValueKey<String>('trailing-${menuEntry.label}');

  for (final TestMenu value in TestMenu.values) {
    final DropdownMenuEntry<TestMenu> entry = DropdownMenuEntry<TestMenu>(
      value: value,
      label: value.label,
      leadingIcon: Icon(key: leadingIconKey(value), Icons.alarm),
      trailingIcon: Icon(key: trailingIconKey(value), Icons.abc),
    );
    menuChildrenWithIcons.add(entry);
  }

  Widget buildTest<T extends Enum>(
    ThemeData themeData,
    List<DropdownMenuEntry<T>> entries, {
    double? width,
    double? menuHeight,
    Widget? leadingIcon,
    Widget? label,
  }) {
    return MaterialApp(
      theme: themeData,
      home: Scaffold(
        body: DropdownMenu<T>(
          label: label,
          leadingIcon: leadingIcon,
          width: width,
          menuHeight: menuHeight,
          dropdownMenuEntries: entries,
        ),
      ),
    );
  }

  Finder findMenuItemButton(String label) {
    // For each menu items there are two MenuItemButton widgets.
    // The last one is the real button item in the menu.
    // The first one is not visible, it is part of _DropdownMenuBody
    // which is used to compute the dropdown width.
    return find.widgetWithText(MenuItemButton, label).last;
  }

  Material getButtonMaterial(WidgetTester tester, String itemLabel) {
    return tester.widget<Material>(
      find.descendant(of: findMenuItemButton(itemLabel), matching: find.byType(Material)),
    );
  }

  bool isItemHighlighted(WidgetTester tester, ThemeData themeData, String itemLabel) {
    final Color? color = getButtonMaterial(tester, itemLabel).color;
    return color == themeData.colorScheme.onSurface.withOpacity(0.12);
  }

  Finder findMenuPanel() {
    return find.byWidgetPredicate((Widget w) => '${w.runtimeType}' == '_MenuPanel');
  }

  Finder findMenuMaterial() {
    return find.descendant(of: findMenuPanel(), matching: find.byType(Material)).first;
  }

  testWidgets('DropdownMenu defaults', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(buildTest(themeData, menuChildren));

    final EditableText editableText = tester.widget(find.byType(EditableText));
    expect(editableText.style.color, themeData.textTheme.bodyLarge!.color);
    expect(editableText.style.background, themeData.textTheme.bodyLarge!.background);
    expect(editableText.style.shadows, themeData.textTheme.bodyLarge!.shadows);
    expect(editableText.style.decoration, themeData.textTheme.bodyLarge!.decoration);
    expect(editableText.style.locale, themeData.textTheme.bodyLarge!.locale);
    expect(editableText.style.wordSpacing, themeData.textTheme.bodyLarge!.wordSpacing);
    expect(editableText.style.fontSize, 16.0);
    expect(editableText.style.height, 1.5);

    final TextField textField = tester.widget(find.byType(TextField));
    expect(textField.decoration?.border, const OutlineInputBorder());
    expect(textField.style?.fontSize, 16.0);
    expect(textField.style?.height, 1.5);

    await tester.tap(find.widgetWithIcon(IconButton, Icons.arrow_drop_down).first);
    await tester.pump();
    expect(find.byType(MenuAnchor), findsOneWidget);

    Material material = tester.widget<Material>(findMenuMaterial());
    expect(material.color, themeData.colorScheme.surfaceContainer);
    expect(material.shadowColor, themeData.colorScheme.shadow);
    expect(material.surfaceTintColor, Colors.transparent);
    expect(material.elevation, 3.0);
    expect(
      material.shape,
      const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4.0))),
    );

    material = getButtonMaterial(tester, TestMenu.mainMenu0.label);
    expect(material.color, Colors.transparent);
    expect(material.elevation, 0.0);
    expect(material.shape, const RoundedRectangleBorder());
    expect(material.textStyle?.color, themeData.colorScheme.onSurface);
    expect(material.textStyle?.fontSize, 14.0);
    expect(material.textStyle?.height, 1.43);
  });

  group('Item style', () {
    const Color focusedBackgroundColor = Color(0xffff0000);
    const Color focusedForegroundColor = Color(0xff00ff00);
    const Color focusedIconColor = Color(0xff0000ff);
    const Color focusedOverlayColor = Color(0xffff00ff);
    const Color defaultBackgroundColor = Color(0xff00ffff);
    const Color defaultForegroundColor = Color(0xff000000);
    const Color defaultIconColor = Color(0xffffffff);
    const Color defaultOverlayColor = Color(0xffffff00);

    final ButtonStyle customButtonStyle = ButtonStyle(
      backgroundColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
        if (states.contains(MaterialState.focused)) {
          return focusedBackgroundColor;
        }
        return defaultBackgroundColor;
      }),
      foregroundColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
        if (states.contains(MaterialState.focused)) {
          return focusedForegroundColor;
        }
        return defaultForegroundColor;
      }),
      iconColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
        if (states.contains(MaterialState.focused)) {
          return focusedIconColor;
        }
        return defaultIconColor;
      }),
      overlayColor: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
        if (states.contains(MaterialState.focused)) {
          return focusedOverlayColor;
        }
        return defaultOverlayColor;
      }),
    );

    final List<DropdownMenuEntry<TestMenu>> styledMenuEntries = <DropdownMenuEntry<TestMenu>>[];
    for (final DropdownMenuEntry<TestMenu> entryWithIcons in menuChildrenWithIcons) {
      styledMenuEntries.add(
        DropdownMenuEntry<TestMenu>(
          value: entryWithIcons.value,
          label: entryWithIcons.label,
          leadingIcon: entryWithIcons.leadingIcon,
          trailingIcon: entryWithIcons.trailingIcon,
          style: customButtonStyle,
        ),
      );
    }

    TextStyle? iconStyle(WidgetTester tester, Key key) {
      final RichText iconRichText = tester.widget<RichText>(
        find.descendant(of: find.byKey(key), matching: find.byType(RichText)).last,
      );
      return iconRichText.text.style;
    }

    RenderObject overlayPainter(WidgetTester tester, TestMenu menuItem) {
      return tester.renderObject(
        find
            .descendant(
              of: findMenuItemButton(menuItem.label),
              matching: find.byElementPredicate(
                (Element element) =>
                    element.renderObject.runtimeType.toString() == '_RenderInkFeatures',
              ),
            )
            .last,
      );
    }

    testWidgets('defaults are correct', (WidgetTester tester) async {
      const TestMenu selectedItem = TestMenu.mainMenu3;
      const TestMenu nonSelectedItem = TestMenu.mainMenu2;

      final ThemeData themeData = ThemeData();
      await tester.pumpWidget(
        MaterialApp(
          theme: themeData,
          home: Scaffold(
            body: DropdownMenu<TestMenu>(
              initialSelection: selectedItem,
              dropdownMenuEntries: menuChildrenWithIcons,
            ),
          ),
        ),
      );

      // Open the menu.
      await tester.tap(find.byType(DropdownMenu<TestMenu>));
      await tester.pump();

      final Material selectedButtonMaterial = getButtonMaterial(tester, selectedItem.label);
      expect(selectedButtonMaterial.color, themeData.colorScheme.onSurface.withOpacity(0.12));
      expect(selectedButtonMaterial.textStyle?.color, themeData.colorScheme.onSurface);
      expect(
        iconStyle(tester, leadingIconKey(selectedItem))?.color,
        themeData.colorScheme.onSurfaceVariant,
      );

      final Material nonSelectedButtonMaterial = getButtonMaterial(tester, nonSelectedItem.label);
      expect(nonSelectedButtonMaterial.color, Colors.transparent);
      expect(nonSelectedButtonMaterial.textStyle?.color, themeData.colorScheme.onSurface);
      expect(
        iconStyle(tester, leadingIconKey(nonSelectedItem))?.color,
        themeData.colorScheme.onSurfaceVariant,
      );

      // Hover the selected item.
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      addTearDown(() async {
        return gesture.removePointer();
      });
      await gesture.addPointer();
      await gesture.moveTo(tester.getCenter(findMenuItemButton(selectedItem.label)));
      await tester.pump();

      expect(
        overlayPainter(tester, selectedItem),
        paints..rect(color: themeData.colorScheme.onSurface.withOpacity(0.1).withAlpha(0)),
      );

      // Hover a non-selected item.
      await gesture.moveTo(tester.getCenter(findMenuItemButton(nonSelectedItem.label)));
      await tester.pump();

      expect(
        overlayPainter(tester, nonSelectedItem),
        paints..rect(color: themeData.colorScheme.onSurface.withOpacity(0.08).withAlpha(0)),
      );
    });

    testWidgets('can be overridden at application theme level', (WidgetTester tester) async {
      const TestMenu selectedItem = TestMenu.mainMenu3;
      const TestMenu nonSelectedItem = TestMenu.mainMenu2;

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(menuButtonTheme: MenuButtonThemeData(style: customButtonStyle)),
          home: Scaffold(
            body: DropdownMenu<TestMenu>(
              initialSelection: selectedItem,
              dropdownMenuEntries: menuChildrenWithIcons,
            ),
          ),
        ),
      );

      // Open the menu.
      await tester.tap(find.byType(DropdownMenu<TestMenu>));
      await tester.pump();

      final Material selectedButtonMaterial = getButtonMaterial(tester, selectedItem.label);
      expect(selectedButtonMaterial.color, focusedBackgroundColor);
      expect(selectedButtonMaterial.textStyle?.color, focusedForegroundColor);
      expect(iconStyle(tester, leadingIconKey(selectedItem))?.color, focusedIconColor);

      final Material nonSelectedButtonMaterial = getButtonMaterial(tester, nonSelectedItem.label);
      expect(nonSelectedButtonMaterial.color, defaultBackgroundColor);
      expect(nonSelectedButtonMaterial.textStyle?.color, defaultForegroundColor);
      expect(iconStyle(tester, leadingIconKey(nonSelectedItem))?.color, defaultIconColor);

      // Hover the selected item.
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      addTearDown(() async {
        return gesture.removePointer();
      });
      await gesture.addPointer();
      await gesture.moveTo(tester.getCenter(findMenuItemButton(selectedItem.label)));
      await tester.pump();

      expect(
        overlayPainter(tester, selectedItem),
        paints..rect(color: focusedOverlayColor.withAlpha(0)),
      );

      // Hover a non-selected item.
      await gesture.moveTo(tester.getCenter(findMenuItemButton(nonSelectedItem.label)));
      await tester.pump();

      expect(
        overlayPainter(tester, nonSelectedItem),
        paints..rect(color: defaultOverlayColor.withAlpha(0)),
      );
    });

    testWidgets('can be overridden at menu entry level', (WidgetTester tester) async {
      const TestMenu selectedItem = TestMenu.mainMenu3;
      const TestMenu nonSelectedItem = TestMenu.mainMenu2;

      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: DropdownMenu<TestMenu>(
              initialSelection: selectedItem,
              dropdownMenuEntries: styledMenuEntries,
            ),
          ),
        ),
      );

      // Open the menu.
      await tester.tap(find.byType(DropdownMenu<TestMenu>));
      await tester.pump();

      final Material selectedButtonMaterial = getButtonMaterial(tester, selectedItem.label);
      expect(selectedButtonMaterial.color, focusedBackgroundColor);
      expect(selectedButtonMaterial.textStyle?.color, focusedForegroundColor);
      expect(iconStyle(tester, leadingIconKey(selectedItem))?.color, focusedIconColor);

      final Material nonSelectedButtonMaterial = getButtonMaterial(tester, nonSelectedItem.label);
      expect(nonSelectedButtonMaterial.color, defaultBackgroundColor);
      expect(nonSelectedButtonMaterial.textStyle?.color, defaultForegroundColor);
      expect(iconStyle(tester, leadingIconKey(nonSelectedItem))?.color, defaultIconColor);

      // Hover the selected item.
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      addTearDown(() async {
        return gesture.removePointer();
      });
      await gesture.addPointer();
      await gesture.moveTo(tester.getCenter(findMenuItemButton(selectedItem.label)));
      await tester.pump();

      expect(
        overlayPainter(tester, selectedItem),
        paints..rect(color: focusedOverlayColor.withAlpha(0)),
      );

      // Hover a non-selected item.
      await gesture.moveTo(tester.getCenter(findMenuItemButton(nonSelectedItem.label)));
      await tester.pump();

      expect(
        overlayPainter(tester, nonSelectedItem),
        paints..rect(color: defaultOverlayColor.withAlpha(0)),
      );
    });

    testWidgets('defined at menu entry level takes precedence', (WidgetTester tester) async {
      const TestMenu selectedItem = TestMenu.mainMenu3;
      const TestMenu nonSelectedItem = TestMenu.mainMenu2;

      const Color luckyColor = Color(0xff777777);
      final ButtonStyle singleColorButtonStyle = ButtonStyle(
        backgroundColor: MaterialStateProperty.all(luckyColor),
        foregroundColor: MaterialStateProperty.all(luckyColor),
        iconColor: MaterialStateProperty.all(luckyColor),
        overlayColor: MaterialStateProperty.all(luckyColor),
      );

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(menuButtonTheme: MenuButtonThemeData(style: singleColorButtonStyle)),
          home: Scaffold(
            body: DropdownMenu<TestMenu>(
              initialSelection: selectedItem,
              dropdownMenuEntries: styledMenuEntries,
            ),
          ),
        ),
      );

      // Open the menu.
      await tester.tap(find.byType(DropdownMenu<TestMenu>));
      await tester.pump();

      final Material selectedButtonMaterial = getButtonMaterial(tester, selectedItem.label);
      expect(selectedButtonMaterial.color, focusedBackgroundColor);
      expect(selectedButtonMaterial.textStyle?.color, focusedForegroundColor);
      expect(iconStyle(tester, leadingIconKey(selectedItem))?.color, focusedIconColor);

      final Material nonSelectedButtonMaterial = getButtonMaterial(tester, nonSelectedItem.label);
      expect(nonSelectedButtonMaterial.color, defaultBackgroundColor);
      expect(nonSelectedButtonMaterial.textStyle?.color, defaultForegroundColor);
      expect(iconStyle(tester, leadingIconKey(nonSelectedItem))?.color, defaultIconColor);

      // Hover the selected item.
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      addTearDown(() async {
        return gesture.removePointer();
      });
      await gesture.addPointer();
      await gesture.moveTo(tester.getCenter(findMenuItemButton(selectedItem.label)));
      await tester.pump();

      expect(
        overlayPainter(tester, selectedItem),
        paints..rect(color: focusedOverlayColor.withAlpha(0)),
      );

      // Hover a non-selected item.
      await gesture.moveTo(tester.getCenter(findMenuItemButton(nonSelectedItem.label)));
      await tester.pump();

      expect(
        overlayPainter(tester, nonSelectedItem),
        paints..rect(color: defaultOverlayColor.withAlpha(0)),
      );
    });

    testWidgets('defined at menu entry level and application level are merged', (
      WidgetTester tester,
    ) async {
      const TestMenu selectedItem = TestMenu.mainMenu3;
      const TestMenu nonSelectedItem = TestMenu.mainMenu2;

      const Color luckyColor = Color(0xff777777);
      final ButtonStyle partialButtonStyle = ButtonStyle(
        backgroundColor: MaterialStateProperty.all(luckyColor),
        foregroundColor: MaterialStateProperty.all(luckyColor),
      );

      final List<DropdownMenuEntry<TestMenu>> partiallyStyledMenuEntries =
          <DropdownMenuEntry<TestMenu>>[];
      for (final DropdownMenuEntry<TestMenu> entryWithIcons in menuChildrenWithIcons) {
        partiallyStyledMenuEntries.add(
          DropdownMenuEntry<TestMenu>(
            value: entryWithIcons.value,
            label: entryWithIcons.label,
            leadingIcon: entryWithIcons.leadingIcon,
            trailingIcon: entryWithIcons.trailingIcon,
            style: partialButtonStyle,
          ),
        );
      }

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(menuButtonTheme: MenuButtonThemeData(style: customButtonStyle)),
          home: Scaffold(
            body: DropdownMenu<TestMenu>(
              initialSelection: selectedItem,
              dropdownMenuEntries: partiallyStyledMenuEntries,
            ),
          ),
        ),
      );

      // Open the menu.
      await tester.tap(find.byType(DropdownMenu<TestMenu>));
      await tester.pump();

      final Material selectedButtonMaterial = getButtonMaterial(tester, selectedItem.label);
      expect(selectedButtonMaterial.color, luckyColor);
      expect(selectedButtonMaterial.textStyle?.color, luckyColor);
      expect(iconStyle(tester, leadingIconKey(selectedItem))?.color, focusedIconColor);

      final Material nonSelectedButtonMaterial = getButtonMaterial(tester, nonSelectedItem.label);
      expect(nonSelectedButtonMaterial.color, luckyColor);
      expect(nonSelectedButtonMaterial.textStyle?.color, luckyColor);
      expect(iconStyle(tester, leadingIconKey(nonSelectedItem))?.color, defaultIconColor);

      // Hover the selected item.
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      addTearDown(() async {
        return gesture.removePointer();
      });
      await gesture.addPointer();
      await gesture.moveTo(tester.getCenter(findMenuItemButton(selectedItem.label)));
      await tester.pump();

      expect(
        overlayPainter(tester, selectedItem),
        paints..rect(color: focusedOverlayColor.withAlpha(0)),
      );

      // Hover a non-selected item.
      await gesture.moveTo(tester.getCenter(findMenuItemButton(nonSelectedItem.label)));
      await tester.pump();

      expect(
        overlayPainter(tester, nonSelectedItem),
        paints..rect(color: defaultOverlayColor.withAlpha(0)),
      );
    });
  });

  testWidgets('Inner TextField is disabled when DropdownMenu is disabled', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: SafeArea(
            child: DropdownMenu<TestMenu>(enabled: false, dropdownMenuEntries: menuChildren),
          ),
        ),
      ),
    );

    final TextField textField = tester.widget(find.byType(TextField));
    expect(textField.enabled, false);
    final Finder menuMaterial = find.ancestor(
      of: find.byType(SingleChildScrollView),
      matching: find.byType(Material),
    );
    expect(menuMaterial, findsNothing);

    await tester.tap(find.byType(TextField));
    await tester.pump();
    final Finder updatedMenuMaterial = find.ancestor(
      of: find.byType(SingleChildScrollView),
      matching: find.byType(Material),
    );
    expect(updatedMenuMaterial, findsNothing);
  });

  testWidgets('Inner IconButton is disabled when DropdownMenu is disabled', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/149598.
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: SafeArea(
            child: DropdownMenu<TestMenu>(enabled: false, dropdownMenuEntries: menuChildren),
          ),
        ),
      ),
    );

    final IconButton trailingButton = tester.widget(
      find.widgetWithIcon(IconButton, Icons.arrow_drop_down).first,
    );
    expect(trailingButton.onPressed, null);
  });

  testWidgets(
    'Material2 - The width of the text field should always be the same as the menu view',
    (WidgetTester tester) async {
      final ThemeData themeData = ThemeData(useMaterial3: false);
      await tester.pumpWidget(
        MaterialApp(
          theme: themeData,
          home: Scaffold(
            body: SafeArea(child: DropdownMenu<TestMenu>(dropdownMenuEntries: menuChildren)),
          ),
        ),
      );

      final Finder textField = find.byType(TextField);
      final Size anchorSize = tester.getSize(textField);
      expect(anchorSize, const Size(180.0, 56.0));

      await tester.tap(find.byType(DropdownMenu<TestMenu>));
      await tester.pumpAndSettle();

      final Finder menuMaterial =
          find
              .ancestor(of: find.byType(SingleChildScrollView), matching: find.byType(Material))
              .first;
      final Size menuSize = tester.getSize(menuMaterial);
      expect(menuSize, const Size(180.0, 304.0));

      // The text field should have same width as the menu
      // when the width property is not null.
      await tester.pumpWidget(buildTest(themeData, menuChildren, width: 200.0));

      final Finder anchor = find.byType(TextField);
      final double width = tester.getSize(anchor).width;
      expect(width, 200.0);

      await tester.tap(anchor);
      await tester.pumpAndSettle();

      final Finder updatedMenu =
          find
              .ancestor(of: find.byType(SingleChildScrollView), matching: find.byType(Material))
              .first;
      final double updatedMenuWidth = tester.getSize(updatedMenu).width;
      expect(updatedMenuWidth, 200.0);
    },
  );

  testWidgets(
    'Material3 - The width of the text field should always be the same as the menu view',
    (WidgetTester tester) async {
      final ThemeData themeData = ThemeData();
      await tester.pumpWidget(
        MaterialApp(
          theme: themeData,
          home: Scaffold(
            body: SafeArea(child: DropdownMenu<TestMenu>(dropdownMenuEntries: menuChildren)),
          ),
        ),
      );

      final Finder textField = find.byType(TextField);
      final double anchorWidth = tester.getSize(textField).width;
      expect(anchorWidth, closeTo(180.5, 0.1));

      await tester.tap(find.byType(DropdownMenu<TestMenu>));
      await tester.pumpAndSettle();

      final Finder menuMaterial =
          find
              .ancestor(of: find.byType(SingleChildScrollView), matching: find.byType(Material))
              .first;
      final double menuWidth = tester.getSize(menuMaterial).width;
      expect(menuWidth, closeTo(180.5, 0.1));

      // The text field should have same width as the menu
      // when the width property is not null.
      await tester.pumpWidget(buildTest(themeData, menuChildren, width: 200.0));

      final Finder anchor = find.byType(TextField);
      final double width = tester.getSize(anchor).width;
      expect(width, 200.0);

      await tester.tap(anchor);
      await tester.pumpAndSettle();

      final Finder updatedMenu =
          find
              .ancestor(of: find.byType(SingleChildScrollView), matching: find.byType(Material))
              .first;
      final double updatedMenuWidth = tester.getSize(updatedMenu).width;
      expect(updatedMenuWidth, 200.0);
    },
  );

  testWidgets('The width property can customize the width of the dropdown menu', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData();
    final List<DropdownMenuEntry<ShortMenu>> shortMenuItems = <DropdownMenuEntry<ShortMenu>>[];

    for (final ShortMenu value in ShortMenu.values) {
      final DropdownMenuEntry<ShortMenu> entry = DropdownMenuEntry<ShortMenu>(
        value: value,
        label: value.label,
      );
      shortMenuItems.add(entry);
    }

    const double customBigWidth = 250.0;
    await tester.pumpWidget(buildTest(themeData, shortMenuItems, width: customBigWidth));
    RenderBox box = tester.firstRenderObject(find.byType(DropdownMenu<ShortMenu>));
    expect(box.size.width, customBigWidth);

    await tester.tap(find.byType(DropdownMenu<ShortMenu>));
    await tester.pump();
    expect(find.byType(MenuItemButton), findsNWidgets(6));
    Size buttonSize = tester.getSize(findMenuItemButton('I0'));
    expect(buttonSize.width, customBigWidth);

    // reset test
    await tester.pumpWidget(Container());
    const double customSmallWidth = 100.0;
    await tester.pumpWidget(buildTest(themeData, shortMenuItems, width: customSmallWidth));
    box = tester.firstRenderObject(find.byType(DropdownMenu<ShortMenu>));
    expect(box.size.width, customSmallWidth);

    await tester.tap(find.byType(DropdownMenu<ShortMenu>));
    await tester.pump();
    expect(find.byType(MenuItemButton), findsNWidgets(6));
    buttonSize = tester.getSize(findMenuItemButton('I0'));
    expect(buttonSize.width, customSmallWidth);
  });

  testWidgets('The width property update test', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/120567
    final ThemeData themeData = ThemeData();
    final List<DropdownMenuEntry<ShortMenu>> shortMenuItems = <DropdownMenuEntry<ShortMenu>>[];

    for (final ShortMenu value in ShortMenu.values) {
      final DropdownMenuEntry<ShortMenu> entry = DropdownMenuEntry<ShortMenu>(
        value: value,
        label: value.label,
      );
      shortMenuItems.add(entry);
    }

    double customWidth = 250.0;
    await tester.pumpWidget(buildTest(themeData, shortMenuItems, width: customWidth));
    RenderBox box = tester.firstRenderObject(find.byType(DropdownMenu<ShortMenu>));
    expect(box.size.width, customWidth);

    // Update width
    customWidth = 400.0;
    await tester.pumpWidget(buildTest(themeData, shortMenuItems, width: customWidth));
    box = tester.firstRenderObject(find.byType(DropdownMenu<ShortMenu>));
    expect(box.size.width, customWidth);
  });

  testWidgets('The width of MenuAnchor respects MenuAnchor.expandedInsets', (
    WidgetTester tester,
  ) async {
    const double parentWidth = 500.0;
    final List<DropdownMenuEntry<ShortMenu>> shortMenuItems = <DropdownMenuEntry<ShortMenu>>[];
    for (final ShortMenu value in ShortMenu.values) {
      final DropdownMenuEntry<ShortMenu> entry = DropdownMenuEntry<ShortMenu>(
        value: value,
        label: value.label,
      );
      shortMenuItems.add(entry);
    }
    Widget buildMenuAnchor({EdgeInsets? expandedInsets}) {
      return MaterialApp(
        home: Scaffold(
          body: SizedBox(
            width: parentWidth,
            height: parentWidth,
            child: DropdownMenu<ShortMenu>(
              expandedInsets: expandedInsets,
              dropdownMenuEntries: shortMenuItems,
            ),
          ),
        ),
      );
    }

    // By default, the width of the text field is determined by the menu children.
    await tester.pumpWidget(buildMenuAnchor());
    RenderBox box = tester.firstRenderObject(find.byType(TextField));
    expect(box.size.width, 136.0);

    await tester.tap(find.byType(TextField));
    await tester.pumpAndSettle();

    Size buttonSize = tester.getSize(findMenuItemButton('I0'));
    expect(buttonSize.width, 136.0);

    // If expandedInsets is EdgeInsets.zero, the width should be the same as its parent.
    await tester.pumpWidget(Container());
    await tester.pumpWidget(buildMenuAnchor(expandedInsets: EdgeInsets.zero));
    box = tester.firstRenderObject(find.byType(TextField));
    expect(box.size.width, parentWidth);

    await tester.tap(find.byType(TextField));
    await tester.pumpAndSettle();

    buttonSize = tester.getSize(findMenuItemButton('I0'));
    expect(buttonSize.width, parentWidth);

    // If expandedInsets is not zero, the width of the text field should be adjusted
    // based on the EdgeInsets.left and EdgeInsets.right. The top and bottom values
    // will be ignored.
    await tester.pumpWidget(Container());
    await tester.pumpWidget(
      buildMenuAnchor(expandedInsets: const EdgeInsets.only(left: 35.0, top: 50.0, right: 20.0)),
    );
    box = tester.firstRenderObject(find.byType(TextField));
    expect(box.size.width, parentWidth - 35.0 - 20.0);
    final Rect containerRect = tester.getRect(find.byType(SizedBox).first);
    final Rect dropdownMenuRect = tester.getRect(find.byType(TextField));
    expect(dropdownMenuRect.top, containerRect.top);

    await tester.tap(find.byType(TextField));
    await tester.pumpAndSettle();

    buttonSize = tester.getSize(findMenuItemButton('I0'));
    expect(buttonSize.width, parentWidth - 35.0 - 20.0);
  });

  // Regression test for https://github.com/flutter/flutter/issues/151769
  testWidgets('expandedInsets can use EdgeInsets or EdgeInsetsDirectional', (
    WidgetTester tester,
  ) async {
    const double parentWidth = 500.0;
    final List<DropdownMenuEntry<ShortMenu>> shortMenuItems = <DropdownMenuEntry<ShortMenu>>[];
    for (final ShortMenu value in ShortMenu.values) {
      final DropdownMenuEntry<ShortMenu> entry = DropdownMenuEntry<ShortMenu>(
        value: value,
        label: value.label,
      );
      shortMenuItems.add(entry);
    }
    Widget buildMenuAnchor({EdgeInsetsGeometry? expandedInsets}) {
      return MaterialApp(
        home: Scaffold(
          body: SizedBox(
            width: parentWidth,
            height: parentWidth,
            child: DropdownMenu<ShortMenu>(
              expandedInsets: expandedInsets,
              dropdownMenuEntries: shortMenuItems,
            ),
          ),
        ),
      );
    }

    // By default, the width of the text field is determined by the menu children.
    await tester.pumpWidget(buildMenuAnchor());
    RenderBox box = tester.firstRenderObject(find.byType(TextField));
    expect(box.size.width, 136.0);

    await tester.tap(find.byType(TextField));
    await tester.pumpAndSettle();

    Size buttonSize = tester.getSize(findMenuItemButton('I0'));
    expect(buttonSize.width, 136.0);

    // If expandedInsets is not zero, the width of the text field should be adjusted
    // based on the EdgeInsets.left and EdgeInsets.right. The top and bottom values
    // will be ignored.
    await tester.pumpWidget(Container());
    await tester.pumpWidget(
      buildMenuAnchor(expandedInsets: const EdgeInsets.only(left: 35.0, top: 50.0, right: 20.0)),
    );
    box = tester.firstRenderObject(find.byType(TextField));
    expect(box.size.width, parentWidth - 35.0 - 20.0);
    Rect containerRect = tester.getRect(find.byType(SizedBox).first);
    Rect dropdownMenuRect = tester.getRect(find.byType(TextField));
    expect(dropdownMenuRect.top, containerRect.top);

    await tester.tap(find.byType(TextField));
    await tester.pumpAndSettle();

    buttonSize = tester.getSize(findMenuItemButton('I0'));
    expect(buttonSize.width, parentWidth - 35.0 - 20.0);

    // Regression test for https://github.com/flutter/flutter/issues/151769.
    // If expandedInsets is not zero, the width of the text field should be adjusted
    // based on the EdgeInsets.end and EdgeInsets.start. The top and bottom values
    // will be ignored.
    await tester.pumpWidget(Container());
    await tester.pumpWidget(
      buildMenuAnchor(
        expandedInsets: const EdgeInsetsDirectional.only(start: 35.0, top: 50.0, end: 20.0),
      ),
    );
    box = tester.firstRenderObject(find.byType(TextField));
    expect(box.size.width, parentWidth - 35.0 - 20.0);
    containerRect = tester.getRect(find.byType(SizedBox).first);
    dropdownMenuRect = tester.getRect(find.byType(TextField));
    expect(dropdownMenuRect.top, containerRect.top);

    await tester.tap(find.byType(TextField));
    await tester.pumpAndSettle();
    buttonSize = tester.getSize(findMenuItemButton('I0'));
    expect(buttonSize.width, parentWidth - 35.0 - 20.0);
  });

  testWidgets(
    'Material2 - The menuHeight property can be used to show a shorter scrollable menu list instead of the complete list',
    (WidgetTester tester) async {
      final ThemeData themeData = ThemeData(useMaterial3: false);
      await tester.pumpWidget(buildTest(themeData, menuChildren));

      await tester.tap(find.byType(DropdownMenu<TestMenu>));
      await tester.pumpAndSettle();

      final Element firstItem = tester.element(findMenuItemButton('Item 0'));
      final RenderBox firstBox = firstItem.renderObject! as RenderBox;
      final Offset topLeft = firstBox.localToGlobal(firstBox.size.topLeft(Offset.zero));
      final Element lastItem = tester.element(findMenuItemButton('Item 5'));
      final RenderBox lastBox = lastItem.renderObject! as RenderBox;
      final Offset bottomRight = lastBox.localToGlobal(lastBox.size.bottomRight(Offset.zero));
      // height = height of MenuItemButton * 6 = 48 * 6
      expect(bottomRight.dy - topLeft.dy, 288.0);

      final Finder menuView =
          find
              .ancestor(of: find.byType(SingleChildScrollView), matching: find.byType(Padding))
              .first;
      final Size menuViewSize = tester.getSize(menuView);
      expect(menuViewSize, const Size(180.0, 304.0)); // 304 = 288 + vertical padding(2 * 8)

      // Constrains the menu height.
      await tester.pumpWidget(Container());
      await tester.pumpWidget(buildTest(themeData, menuChildren, menuHeight: 100));
      await tester.pumpAndSettle();

      await tester.tap(find.byType(DropdownMenu<TestMenu>));
      await tester.pumpAndSettle();

      final Finder updatedMenu =
          find
              .ancestor(of: find.byType(SingleChildScrollView), matching: find.byType(Padding))
              .first;

      final Size updatedMenuSize = tester.getSize(updatedMenu);
      expect(updatedMenuSize, const Size(180.0, 100.0));
    },
  );

  testWidgets(
    'Material3 - The menuHeight property can be used to show a shorter scrollable menu list instead of the complete list',
    (WidgetTester tester) async {
      final ThemeData themeData = ThemeData();
      await tester.pumpWidget(buildTest(themeData, menuChildren));

      await tester.tap(find.byType(DropdownMenu<TestMenu>));
      await tester.pumpAndSettle();

      final Element firstItem = tester.element(findMenuItemButton('Item 0'));
      final RenderBox firstBox = firstItem.renderObject! as RenderBox;
      final Offset topLeft = firstBox.localToGlobal(firstBox.size.topLeft(Offset.zero));
      final Element lastItem = tester.element(findMenuItemButton('Item 5'));
      final RenderBox lastBox = lastItem.renderObject! as RenderBox;
      final Offset bottomRight = lastBox.localToGlobal(lastBox.size.bottomRight(Offset.zero));
      // height = height of MenuItemButton * 6 = 48 * 6
      expect(bottomRight.dy - topLeft.dy, 288.0);

      final Finder menuView =
          find
              .ancestor(of: find.byType(SingleChildScrollView), matching: find.byType(Padding))
              .first;
      final Size menuViewSize = tester.getSize(menuView);
      expect(menuViewSize.width, closeTo(180.6, 0.1));
      expect(menuViewSize.height, equals(304.0)); // 304 = 288 + vertical padding(2 * 8)

      // Constrains the menu height.
      await tester.pumpWidget(Container());
      await tester.pumpWidget(buildTest(themeData, menuChildren, menuHeight: 100));
      await tester.pumpAndSettle();

      await tester.tap(find.byType(DropdownMenu<TestMenu>));
      await tester.pumpAndSettle();

      final Finder updatedMenu =
          find
              .ancestor(of: find.byType(SingleChildScrollView), matching: find.byType(Padding))
              .first;

      final Size updatedMenuSize = tester.getSize(updatedMenu);
      expect(updatedMenuSize.width, closeTo(180.6, 0.1));
      expect(updatedMenuSize.height, equals(100.0));
    },
  );

  testWidgets('The text in the menu button should be aligned with the text of '
      'the text field - LTR', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();
    // Default text field (without leading icon).
    await tester.pumpWidget(buildTest(themeData, menuChildren, label: const Text('label')));

    final Finder label = find.text('label');
    final Offset labelTopLeft = tester.getTopLeft(label);

    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pumpAndSettle();
    final Finder itemText = find.text('Item 0').last;
    final Offset itemTextTopLeft = tester.getTopLeft(itemText);

    expect(labelTopLeft.dx, equals(itemTextTopLeft.dx));

    // Test when the text field has a leading icon.
    await tester.pumpWidget(Container());
    await tester.pumpWidget(
      buildTest(
        themeData,
        menuChildren,
        leadingIcon: const Icon(Icons.search),
        label: const Text('label'),
      ),
    );

    final Finder leadingIcon = find.widgetWithIcon(SizedBox, Icons.search).last;
    final double iconWidth = tester.getSize(leadingIcon).width;
    final Finder updatedLabel = find.text('label');
    final Offset updatedLabelTopLeft = tester.getTopLeft(updatedLabel);

    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pumpAndSettle();
    final Finder updatedItemText = find.text('Item 0').last;
    final Offset updatedItemTextTopLeft = tester.getTopLeft(updatedItemText);

    expect(updatedLabelTopLeft.dx, equals(updatedItemTextTopLeft.dx));
    expect(updatedLabelTopLeft.dx, equals(iconWidth + leadingIconToInputPadding));

    // Test when then leading icon is a widget with a bigger size.
    await tester.pumpWidget(Container());
    await tester.pumpWidget(
      buildTest(
        themeData,
        menuChildren,
        leadingIcon: const SizedBox(width: 75.0, child: Icon(Icons.search)),
        label: const Text('label'),
      ),
    );

    final Finder largeLeadingIcon = find.widgetWithIcon(SizedBox, Icons.search).last;
    final double largeIconWidth = tester.getSize(largeLeadingIcon).width;
    final Finder updatedLabel1 = find.text('label');
    final Offset updatedLabelTopLeft1 = tester.getTopLeft(updatedLabel1);

    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pumpAndSettle();
    final Finder updatedItemText1 = find.text('Item 0').last;
    final Offset updatedItemTextTopLeft1 = tester.getTopLeft(updatedItemText1);

    expect(updatedLabelTopLeft1.dx, equals(updatedItemTextTopLeft1.dx));
    expect(updatedLabelTopLeft1.dx, equals(largeIconWidth + leadingIconToInputPadding));
  });

  testWidgets('The text in the menu button should be aligned with the text of '
      'the text field - RTL', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();
    // Default text field (without leading icon).
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: Directionality(
            textDirection: TextDirection.rtl,
            child: DropdownMenu<TestMenu>(
              label: const Text('label'),
              dropdownMenuEntries: menuChildren,
            ),
          ),
        ),
      ),
    );

    final Finder label = find.text('label');
    final Offset labelTopRight = tester.getTopRight(label);

    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pumpAndSettle();
    final Finder itemText = find.text('Item 0').last;
    final Offset itemTextTopRight = tester.getTopRight(itemText);

    expect(labelTopRight.dx, equals(itemTextTopRight.dx));

    // Test when the text field has a leading icon.
    await tester.pumpWidget(Container());
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: Directionality(
            textDirection: TextDirection.rtl,
            child: DropdownMenu<TestMenu>(
              leadingIcon: const Icon(Icons.search),
              label: const Text('label'),
              dropdownMenuEntries: menuChildren,
            ),
          ),
        ),
      ),
    );
    await tester.pump();

    final Finder leadingIcon = find.widgetWithIcon(SizedBox, Icons.search).last;
    final double iconWidth = tester.getSize(leadingIcon).width;
    final Offset dropdownMenuTopRight = tester.getTopRight(find.byType(DropdownMenu<TestMenu>));
    final Finder updatedLabel = find.text('label');
    final Offset updatedLabelTopRight = tester.getTopRight(updatedLabel);

    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pumpAndSettle();
    final Finder updatedItemText = find.text('Item 0').last;
    final Offset updatedItemTextTopRight = tester.getTopRight(updatedItemText);

    expect(updatedLabelTopRight.dx, equals(updatedItemTextTopRight.dx));
    expect(
      updatedLabelTopRight.dx,
      equals(dropdownMenuTopRight.dx - iconWidth - leadingIconToInputPadding),
    );

    // Test when then leading icon is a widget with a bigger size.
    await tester.pumpWidget(Container());
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: Directionality(
            textDirection: TextDirection.rtl,
            child: DropdownMenu<TestMenu>(
              leadingIcon: const SizedBox(width: 75.0, child: Icon(Icons.search)),
              label: const Text('label'),
              dropdownMenuEntries: menuChildren,
            ),
          ),
        ),
      ),
    );
    await tester.pump();

    final Finder largeLeadingIcon = find.widgetWithIcon(SizedBox, Icons.search).last;
    final double largeIconWidth = tester.getSize(largeLeadingIcon).width;
    final Offset updatedDropdownMenuTopRight = tester.getTopRight(
      find.byType(DropdownMenu<TestMenu>),
    );
    final Finder updatedLabel1 = find.text('label');
    final Offset updatedLabelTopRight1 = tester.getTopRight(updatedLabel1);

    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pumpAndSettle();
    final Finder updatedItemText1 = find.text('Item 0').last;
    final Offset updatedItemTextTopRight1 = tester.getTopRight(updatedItemText1);

    expect(updatedLabelTopRight1.dx, equals(updatedItemTextTopRight1.dx));
    expect(
      updatedLabelTopRight1.dx,
      equals(updatedDropdownMenuTopRight.dx - largeIconWidth - leadingIconToInputPadding),
    );
  });

  testWidgets('DropdownMenu has default trailing icon button', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(buildTest(themeData, menuChildren));
    await tester.pump();

    final Finder iconButton = find.widgetWithIcon(IconButton, Icons.arrow_drop_down).first;
    expect(iconButton, findsOneWidget);

    await tester.tap(iconButton);
    await tester.pump();

    final Finder menuMaterial =
        find
            .ancestor(
              of: findMenuItemButton(TestMenu.mainMenu0.label),
              matching: find.byType(Material),
            )
            .last;
    expect(menuMaterial, findsOneWidget);
  });

  testWidgets('Trailing IconButton status test', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(buildTest(themeData, menuChildren, width: 100.0, menuHeight: 100.0));
    await tester.pump();

    Finder iconButton = find.widgetWithIcon(IconButton, Icons.arrow_drop_up);
    expect(iconButton, findsNothing);
    iconButton = find.widgetWithIcon(IconButton, Icons.arrow_drop_down).first;
    expect(iconButton, findsOneWidget);

    await tester.tap(iconButton);
    await tester.pump();

    iconButton = find.widgetWithIcon(IconButton, Icons.arrow_drop_up).first;
    expect(iconButton, findsOneWidget);
    iconButton = find.widgetWithIcon(IconButton, Icons.arrow_drop_down);
    expect(iconButton, findsNothing);

    // Tap outside
    await tester.tapAt(const Offset(500.0, 500.0));
    await tester.pump();

    iconButton = find.widgetWithIcon(IconButton, Icons.arrow_drop_up);
    expect(iconButton, findsNothing);
    iconButton = find.widgetWithIcon(IconButton, Icons.arrow_drop_down).first;
    expect(iconButton, findsOneWidget);
  });

  testWidgets('Do not crash when resize window during menu opening', (WidgetTester tester) async {
    addTearDown(tester.view.reset);
    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return DropdownMenu<TestMenu>(
                width: MediaQuery.of(context).size.width,
                dropdownMenuEntries: menuChildren,
              );
            },
          ),
        ),
      ),
    );

    final Finder iconButton = find.widgetWithIcon(IconButton, Icons.arrow_drop_down).first;
    expect(iconButton, findsOneWidget);

    await tester.tap(iconButton);
    await tester.pump();

    expect(findMenuItemButton(TestMenu.mainMenu0.label), findsOne);

    // didChangeMetrics
    tester.view.physicalSize = const Size(700.0, 700.0);
    await tester.pump();

    // Go without throw.
  });

  testWidgets('DropdownMenu can customize trailing icon button', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: DropdownMenu<TestMenu>(
            trailingIcon: const Icon(Icons.ac_unit),
            dropdownMenuEntries: menuChildren,
          ),
        ),
      ),
    );
    await tester.pump();

    final Finder iconButton = find.widgetWithIcon(IconButton, Icons.ac_unit).first;
    expect(iconButton, findsOneWidget);

    await tester.tap(iconButton);
    await tester.pump();

    final Finder menuMaterial =
        find
            .ancestor(
              of: findMenuItemButton(TestMenu.mainMenu0.label),
              matching: find.byType(Material),
            )
            .last;
    expect(menuMaterial, findsOneWidget);
  });

  testWidgets('Down key can highlight the menu item while focused', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: DropdownMenu<TestMenu>(
            requestFocusOnTap: true,
            trailingIcon: const Icon(Icons.ac_unit),
            dropdownMenuEntries: menuChildren,
          ),
        ),
      ),
    );

    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();
    expect(isItemHighlighted(tester, themeData, 'Item 0'), true);

    // Press down key one more time, the highlight should move to the next item.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();
    expect(isItemHighlighted(tester, themeData, 'Menu 1'), true);

    // The previous item should not be highlighted.
    expect(isItemHighlighted(tester, themeData, 'Item 0'), false);
  });

  testWidgets('Up key can highlight the menu item while focused', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: DropdownMenu<TestMenu>(requestFocusOnTap: true, dropdownMenuEntries: menuChildren),
        ),
      ),
    );

    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pump();
    expect(isItemHighlighted(tester, themeData, 'Item 5'), true);

    // Press up key one more time, the highlight should move up to the item 4.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pump();
    expect(isItemHighlighted(tester, themeData, 'Item 4'), true);

    // The previous item should not be highlighted.
    expect(isItemHighlighted(tester, themeData, 'Item 5'), false);
  });

  testWidgets('Left and right keys can move text field selection', (WidgetTester tester) async {
    final TextEditingController controller = TextEditingController();
    addTearDown(controller.dispose);

    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: DropdownMenu<TestMenu>(
            requestFocusOnTap: true,
            enableFilter: true,
            filterCallback: (List<DropdownMenuEntry<TestMenu>> entries, String filter) {
              return entries
                  .where((DropdownMenuEntry<TestMenu> element) => element.label.contains(filter))
                  .toList();
            },
            dropdownMenuEntries: menuChildren,
            controller: controller,
          ),
        ),
      ),
    );

    // Open the menu.
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();

    await tester.enterText(find.byType(TextField).first, 'example');
    await tester.pump();
    expect(controller.text, 'example');
    expect(controller.selection, const TextSelection.collapsed(offset: 7));

    // Press left key, the caret should move left.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pump();
    expect(controller.selection, const TextSelection.collapsed(offset: 6));

    // Press Right key, the caret should move right.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();
    expect(controller.selection, const TextSelection.collapsed(offset: 7));
  });

  // Regression test for https://github.com/flutter/flutter/issues/156712.
  testWidgets('Up and down keys can highlight the menu item when expandedInsets is set', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: DropdownMenu<TestMenu>(
            expandedInsets: EdgeInsets.zero,
            requestFocusOnTap: true,
            dropdownMenuEntries: menuChildren,
          ),
        ),
      ),
    );

    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pump();
    expect(isItemHighlighted(tester, themeData, 'Item 5'), true);

    // Press up key one more time, the highlight should move up to the item 4.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pump();
    expect(isItemHighlighted(tester, themeData, 'Item 4'), true);

    // The previous item should not be highlighted.
    expect(isItemHighlighted(tester, themeData, 'Item 5'), false);

    // Press down key, the highlight should move back to the item 5.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();
    expect(isItemHighlighted(tester, themeData, 'Item 5'), true);
  });

  // Regression test for https://github.com/flutter/flutter/issues/156712.
  testWidgets('Left and right keys can move text field selection when expandedInsets is set', (
    WidgetTester tester,
  ) async {
    final TextEditingController controller = TextEditingController();
    addTearDown(controller.dispose);

    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: DropdownMenu<TestMenu>(
            expandedInsets: EdgeInsets.zero,
            requestFocusOnTap: true,
            enableFilter: true,
            filterCallback: (List<DropdownMenuEntry<TestMenu>> entries, String filter) {
              return entries
                  .where((DropdownMenuEntry<TestMenu> element) => element.label.contains(filter))
                  .toList();
            },
            dropdownMenuEntries: menuChildren,
            controller: controller,
          ),
        ),
      ),
    );

    // Open the menu.
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();

    await tester.enterText(find.byType(TextField).first, 'example');
    await tester.pump();
    expect(controller.text, 'example');
    expect(controller.selection, const TextSelection.collapsed(offset: 7));

    // Press left key, the caret should move left.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowLeft);
    await tester.pump();
    expect(controller.selection, const TextSelection.collapsed(offset: 6));

    // Press Right key, the caret should move right.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowRight);
    await tester.pump();
    expect(controller.selection, const TextSelection.collapsed(offset: 7));
  });

  // Regression test for https://github.com/flutter/flutter/issues/147253.
  testWidgets('Down key and up key can navigate while focused when a label text '
      'contains another label text', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: const Scaffold(
          body: DropdownMenu<int>(
            requestFocusOnTap: true,
            dropdownMenuEntries: <DropdownMenuEntry<int>>[
              DropdownMenuEntry<int>(value: 0, label: 'ABC'),
              DropdownMenuEntry<int>(value: 1, label: 'AB'),
              DropdownMenuEntry<int>(value: 2, label: 'ABCD'),
            ],
          ),
        ),
      ),
    );

    await tester.tap(find.byType(DropdownMenu<int>));
    await tester.pump();

    // Press down key three times, the highlight should move to the next item each time.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();
    expect(isItemHighlighted(tester, themeData, 'ABC'), true);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();
    expect(isItemHighlighted(tester, themeData, 'AB'), true);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();
    expect(isItemHighlighted(tester, themeData, 'ABCD'), true);

    // Press up key two times, the highlight should up each time.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pump();
    expect(isItemHighlighted(tester, themeData, 'AB'), true);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pump();
    expect(isItemHighlighted(tester, themeData, 'ABC'), true);
  });

  // Regression test for https://github.com/flutter/flutter/issues/151878.
  testWidgets('Searching for non matching item does not crash', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenu<TestMenu>(
            enableFilter: true,
            requestFocusOnTap: true,
            dropdownMenuEntries: menuChildren,
          ),
        ),
      ),
    );

    // Open the menu.
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();
    await tester.enterText(find.byType(TextField).first, 'Me');
    await tester.pump();
    await tester.enterText(find.byType(TextField).first, 'Meu');
    await tester.pump();
    expect(tester.takeException(), isNull);
  });

  // Regression test for https://github.com/flutter/flutter/issues/154532.
  testWidgets('Keyboard navigation does not throw when no entries match the filter', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenu<TestMenu>(
            requestFocusOnTap: true,
            enableFilter: true,
            dropdownMenuEntries: menuChildren,
          ),
        ),
      ),
    );
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();
    await tester.enterText(find.byType(TextField).first, 'No match');
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();
    await tester.enterText(find.byType(TextField).first, 'No match 2');
    await tester.pump();
    expect(tester.takeException(), isNull);
  });

  // Regression test for https://github.com/flutter/flutter/issues/147253.
  testWidgets('Default search prioritises the current highlight', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: DropdownMenu<TestMenu>(requestFocusOnTap: true, dropdownMenuEntries: menuChildren),
        ),
      ),
    );

    const String itemLabel = 'Item 2';
    // Open the menu
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();
    // Highlight the third item by exact search.
    await tester.enterText(find.byType(TextField).first, itemLabel);
    await tester.pump();
    expect(isItemHighlighted(tester, themeData, itemLabel), true);

    // Search something that matches multiple items.
    await tester.enterText(find.byType(TextField).first, 'Item');
    await tester.pump();
    // The third item should still be highlighted.
    expect(isItemHighlighted(tester, themeData, itemLabel), true);
  });

  // Regression test for https://github.com/flutter/flutter/issues/152375.
  testWidgets('Down key and up key can navigate while focused when a label text contains '
      'another label text using customized search algorithm', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: DropdownMenu<int>(
            requestFocusOnTap: true,
            searchCallback: (List<DropdownMenuEntry<int>> entries, String query) {
              if (query.isEmpty) {
                return null;
              }
              final int index = entries.indexWhere(
                (DropdownMenuEntry<int> entry) => entry.label.contains(query),
              );
              return index != -1 ? index : null;
            },
            dropdownMenuEntries: const <DropdownMenuEntry<int>>[
              DropdownMenuEntry<int>(value: 0, label: 'ABC'),
              DropdownMenuEntry<int>(value: 1, label: 'AB'),
              DropdownMenuEntry<int>(value: 2, label: 'ABCD'),
            ],
          ),
        ),
      ),
    );

    await tester.tap(find.byType(DropdownMenu<int>));
    await tester.pump();

    // Press down key three times, the highlight should move to the next item each time.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();
    expect(isItemHighlighted(tester, themeData, 'ABC'), true);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();
    expect(isItemHighlighted(tester, themeData, 'AB'), true);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();
    expect(isItemHighlighted(tester, themeData, 'ABCD'), true);

    // Press up key two times, the highlight should up each time.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pump();
    expect(isItemHighlighted(tester, themeData, 'AB'), true);

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pump();
    expect(isItemHighlighted(tester, themeData, 'ABC'), true);
  });

  // Regression test for https://github.com/flutter/flutter/issues/152375.
  testWidgets('Searching can highlight entry after keyboard navigation while focused', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: DropdownMenu<TestMenu>(requestFocusOnTap: true, dropdownMenuEntries: menuChildren),
        ),
      ),
    );

    // Open the menu and highlight the first item.
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();
    // Search for the last item.
    final String searchedLabel = menuChildren.last.label;
    await tester.enterText(find.byType(TextField).first, searchedLabel);
    await tester.pump();
    // The corresponding menu entry is highlighted.
    expect(isItemHighlighted(tester, themeData, searchedLabel), true);
  });

  testWidgets('The text input should match the label of the menu item '
      'when pressing down key while focused', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: DropdownMenu<TestMenu>(requestFocusOnTap: true, dropdownMenuEntries: menuChildren),
        ),
      ),
    );

    // Open the menu
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();
    expect(find.widgetWithText(TextField, 'Item 0'), findsOneWidget);

    // Press down key one more time to the next item.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();
    expect(find.widgetWithText(TextField, 'Menu 1'), findsOneWidget);

    // Press down to the next item.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();
    expect(find.widgetWithText(TextField, 'Item 2'), findsOneWidget);
  });

  testWidgets('The text input should match the label of the menu item '
      'when pressing up key while focused', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: DropdownMenu<TestMenu>(requestFocusOnTap: true, dropdownMenuEntries: menuChildren),
        ),
      ),
    );

    // Open the menu
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pump();
    expect(find.widgetWithText(TextField, 'Item 5'), findsOneWidget);

    // Press up key one more time to the upper item.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pump();
    expect(find.widgetWithText(TextField, 'Item 4'), findsOneWidget);

    // Press up to the upper item.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowUp);
    await tester.pump();
    expect(find.widgetWithText(TextField, 'Item 3'), findsOneWidget);
  });

  testWidgets('Disabled button will be skipped while pressing up/down key while focused', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData();
    final List<DropdownMenuEntry<TestMenu>> menuWithDisabledItems = <DropdownMenuEntry<TestMenu>>[
      const DropdownMenuEntry<TestMenu>(value: TestMenu.mainMenu0, label: 'Item 0'),
      const DropdownMenuEntry<TestMenu>(value: TestMenu.mainMenu1, label: 'Item 1', enabled: false),
      const DropdownMenuEntry<TestMenu>(value: TestMenu.mainMenu2, label: 'Item 2', enabled: false),
      const DropdownMenuEntry<TestMenu>(value: TestMenu.mainMenu3, label: 'Item 3'),
      const DropdownMenuEntry<TestMenu>(value: TestMenu.mainMenu4, label: 'Item 4'),
      const DropdownMenuEntry<TestMenu>(value: TestMenu.mainMenu5, label: 'Item 5', enabled: false),
    ];
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: DropdownMenu<TestMenu>(
            requestFocusOnTap: true,
            dropdownMenuEntries: menuWithDisabledItems,
          ),
        ),
      ),
    );
    await tester.pump();

    // Open the menu
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pumpAndSettle();

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pumpAndSettle();

    // First item is highlighted as it's enabled.
    expect(isItemHighlighted(tester, themeData, 'Item 0'), true);

    // Continue to press down key. Item 3 should be highlighted as Menu 1 and Item 2 are both disabled.
    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pumpAndSettle();
    expect(isItemHighlighted(tester, themeData, 'Item 3'), true);
  });

  testWidgets('Searching is enabled by default if initialSelection is non null', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: DropdownMenu<TestMenu>(
            requestFocusOnTap: true,
            initialSelection: TestMenu.mainMenu1,
            dropdownMenuEntries: menuChildren,
          ),
        ),
      ),
    );

    // Open the menu
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();

    // Initial selection (Menu 1) button is highlighted.
    expect(isItemHighlighted(tester, themeData, 'Menu 1'), true);
  });

  testWidgets('Highlight can move up/down starting from the searching result while focused', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: DropdownMenu<TestMenu>(requestFocusOnTap: true, dropdownMenuEntries: menuChildren),
        ),
      ),
    );

    // Open the menu
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();
    await tester.enterText(find.byType(TextField).first, 'Menu 1');
    await tester.pumpAndSettle();
    expect(isItemHighlighted(tester, themeData, 'Menu 1'), true);

    // Press up to the upper item (Item 0).
    await simulateKeyDownEvent(LogicalKeyboardKey.arrowUp);
    await simulateKeyUpEvent(LogicalKeyboardKey.arrowUp);
    await tester.pumpAndSettle();
    expect(find.widgetWithText(TextField, 'Item 0'), findsOneWidget);
    expect(isItemHighlighted(tester, themeData, 'Item 0'), true);

    // Continue to move up to the last item (Item 5).
    await simulateKeyDownEvent(LogicalKeyboardKey.arrowUp);
    await simulateKeyUpEvent(LogicalKeyboardKey.arrowUp);
    await tester.pumpAndSettle();
    expect(find.widgetWithText(TextField, 'Item 5'), findsOneWidget);
    expect(isItemHighlighted(tester, themeData, 'Item 5'), true);
  });

  testWidgets('Filtering is disabled by default', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: DropdownMenu<TestMenu>(requestFocusOnTap: true, dropdownMenuEntries: menuChildren),
        ),
      ),
    );

    // Open the menu
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();

    await tester.enterText(find.byType(TextField).first, 'Menu 1');
    await tester.pumpAndSettle();
    for (final TestMenu menu in TestMenu.values) {
      // One is layout for the _DropdownMenuBody, the other one is the real button item in the menu.
      expect(find.widgetWithText(MenuItemButton, menu.label), findsNWidgets(2));
    }
  });

  testWidgets('Enable filtering', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: DropdownMenu<TestMenu>(
            requestFocusOnTap: true,
            enableFilter: true,
            dropdownMenuEntries: menuChildren,
          ),
        ),
      ),
    );

    // Open the menu
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();

    await tester.enterText(find.byType(TextField).first, 'Menu 1');
    await tester.pumpAndSettle();
    for (final TestMenu menu in TestMenu.values) {
      // 'Menu 1' should be 2, other items should only find one.
      if (menu.label == TestMenu.mainMenu1.label) {
        expect(find.widgetWithText(MenuItemButton, menu.label), findsNWidgets(2));
      } else {
        expect(find.widgetWithText(MenuItemButton, menu.label), findsOneWidget);
      }
    }
  });

  testWidgets('Enable filtering with custom filter callback that filter text case sensitive', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData();
    final TextEditingController controller = TextEditingController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: DropdownMenu<TestMenu>(
            requestFocusOnTap: true,
            enableFilter: true,
            filterCallback: (List<DropdownMenuEntry<TestMenu>> entries, String filter) {
              return entries
                  .where((DropdownMenuEntry<TestMenu> element) => element.label.contains(filter))
                  .toList();
            },
            dropdownMenuEntries: menuChildren,
            controller: controller,
          ),
        ),
      ),
    );

    // Open the menu.
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();

    await tester.enterText(find.byType(TextField).first, 'item');
    expect(controller.text, 'item');
    await tester.pumpAndSettle();
    for (final TestMenu menu in TestMenu.values) {
      expect(findMenuItemButton(menu.label).hitTestable(), findsNothing);
    }

    await tester.enterText(find.byType(TextField).first, 'Item');
    expect(controller.text, 'Item');
    await tester.pumpAndSettle();
    expect(findMenuItemButton('Item 0').hitTestable(), findsOneWidget);
    expect(findMenuItemButton('Menu 1').hitTestable(), findsNothing);
    expect(findMenuItemButton('Item 2').hitTestable(), findsOneWidget);
    expect(findMenuItemButton('Item 3').hitTestable(), findsOneWidget);
    expect(findMenuItemButton('Item 4').hitTestable(), findsOneWidget);
    expect(findMenuItemButton('Item 5').hitTestable(), findsOneWidget);
  });

  testWidgets(
    'Throw assertion error when enable filtering with custom filter callback and enableFilter set on False',
    (WidgetTester tester) async {
      final ThemeData themeData = ThemeData();
      final TextEditingController controller = TextEditingController();
      addTearDown(controller.dispose);

      expect(() {
        MaterialApp(
          theme: themeData,
          home: Scaffold(
            body: DropdownMenu<TestMenu>(
              requestFocusOnTap: true,
              filterCallback: (List<DropdownMenuEntry<TestMenu>> entries, String filter) {
                return entries
                    .where((DropdownMenuEntry<TestMenu> element) => element.label.contains(filter))
                    .toList();
              },
              dropdownMenuEntries: menuChildren,
              controller: controller,
            ),
          ),
        );
      }, throwsAssertionError);
    },
  );

  testWidgets('The controller can access the value in the input field', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData();
    final TextEditingController controller = TextEditingController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Scaffold(
              body: DropdownMenu<TestMenu>(
                requestFocusOnTap: true,
                enableFilter: true,
                dropdownMenuEntries: menuChildren,
                controller: controller,
              ),
            );
          },
        ),
      ),
    );

    // Open the menu
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();
    final Finder item3 = findMenuItemButton('Item 3');
    await tester.tap(item3);
    await tester.pumpAndSettle();

    expect(controller.text, 'Item 3');

    await tester.enterText(find.byType(TextField).first, 'New Item');
    expect(controller.text, 'New Item');
  });

  testWidgets('The menu should be closed after text editing is complete', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData();
    final TextEditingController controller = TextEditingController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: DropdownMenu<TestMenu>(
            requestFocusOnTap: true,
            enableFilter: true,
            dropdownMenuEntries: menuChildren,
            controller: controller,
          ),
        ),
      ),
    );
    // Access the MenuAnchor
    final MenuAnchor menuAnchor = tester.widget<MenuAnchor>(find.byType(MenuAnchor));

    // Open the menu
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pumpAndSettle();
    expect(menuAnchor.controller!.isOpen, true);

    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pumpAndSettle();
    expect(menuAnchor.controller!.isOpen, false);
  });

  testWidgets('The onSelected gets called only when a selection is made', (
    WidgetTester tester,
  ) async {
    int selectionCount = 0;

    final ThemeData themeData = ThemeData();
    final List<DropdownMenuEntry<TestMenu>> menuWithDisabledItems = <DropdownMenuEntry<TestMenu>>[
      const DropdownMenuEntry<TestMenu>(value: TestMenu.mainMenu0, label: 'Item 0'),
      const DropdownMenuEntry<TestMenu>(value: TestMenu.mainMenu0, label: 'Item 1', enabled: false),
      const DropdownMenuEntry<TestMenu>(value: TestMenu.mainMenu0, label: 'Item 2'),
      const DropdownMenuEntry<TestMenu>(value: TestMenu.mainMenu0, label: 'Item 3'),
    ];
    final TextEditingController controller = TextEditingController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Scaffold(
              body: DropdownMenu<TestMenu>(
                dropdownMenuEntries: menuWithDisabledItems,
                controller: controller,
                onSelected: (_) {
                  setState(() {
                    selectionCount++;
                  });
                },
              ),
            );
          },
        ),
      ),
    );

    // Open the menu
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();
    final bool isMobile = switch (themeData.platform) {
      TargetPlatform.android || TargetPlatform.iOS || TargetPlatform.fuchsia => true,
      TargetPlatform.macOS || TargetPlatform.linux || TargetPlatform.windows => false,
    };
    int expectedCount = 1;

    // Test onSelected on key press
    await simulateKeyDownEvent(LogicalKeyboardKey.arrowDown);
    await tester.pumpAndSettle();
    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pumpAndSettle();
    expect(selectionCount, expectedCount);

    // Open the menu
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();

    // Disabled item doesn't trigger onSelected callback.
    final Finder item1 = findMenuItemButton('Item 1');
    await tester.tap(item1);
    await tester.pumpAndSettle();

    expect(controller.text, 'Item 0');
    expect(selectionCount, expectedCount);

    final Finder item2 = findMenuItemButton('Item 2');
    await tester.tap(item2);
    await tester.pumpAndSettle();

    expect(controller.text, 'Item 2');
    expect(selectionCount, ++expectedCount);

    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();
    final Finder item3 = findMenuItemButton('Item 3');
    await tester.tap(item3);
    await tester.pumpAndSettle();

    expect(controller.text, 'Item 3');
    expect(selectionCount, ++expectedCount);

    // On desktop platforms, when typing something in the text field without selecting any of the options,
    // the onSelected should not be called.
    if (!isMobile) {
      await tester.enterText(find.byType(TextField).first, 'New Item');
      expect(controller.text, 'New Item');
      expect(selectionCount, expectedCount);
      expect(find.widgetWithText(TextField, 'New Item'), findsOneWidget);
      await tester.enterText(find.byType(TextField).first, '');
      expect(selectionCount, expectedCount);
      expect(controller.text.isEmpty, true);
    }
  }, variant: TargetPlatformVariant.all());

  testWidgets('The selectedValue gives an initial text and highlights the according item', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData();
    final TextEditingController controller = TextEditingController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Scaffold(
              body: DropdownMenu<TestMenu>(
                initialSelection: TestMenu.mainMenu3,
                dropdownMenuEntries: menuChildren,
                controller: controller,
              ),
            );
          },
        ),
      ),
    );

    expect(find.widgetWithText(TextField, 'Item 3'), findsOneWidget);

    // Open the menu
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();

    // Validate the item 3 is highlighted.
    expect(isItemHighlighted(tester, themeData, 'Item 3'), true);
  });

  testWidgets(
    'When the initial selection matches a menu entry, the text field displays the corresponding value',
    (WidgetTester tester) async {
      final TextEditingController controller = TextEditingController();
      addTearDown(controller.dispose);

      await tester.pumpWidget(
        MaterialApp(
          home: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Scaffold(
                body: DropdownMenu<TestMenu>(
                  initialSelection: TestMenu.mainMenu3,
                  dropdownMenuEntries: menuChildren,
                  controller: controller,
                ),
              );
            },
          ),
        ),
      );

      expect(controller.text, TestMenu.mainMenu3.label);
    },
  );

  testWidgets('Text field is empty when the initial selection does not match any menu entries', (
    WidgetTester tester,
  ) async {
    final TextEditingController controller = TextEditingController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Scaffold(
              body: DropdownMenu<TestMenu>(
                initialSelection: TestMenu.mainMenu3,
                // Use a menu entries which does not contain TestMenu.mainMenu3.
                dropdownMenuEntries: menuChildren.getRange(0, 1).toList(),
                controller: controller,
              ),
            );
          },
        ),
      ),
    );

    expect(controller.text, isEmpty);
  });

  testWidgets(
    'Text field content is not cleared when the initial selection does not match any menu entries',
    (WidgetTester tester) async {
      final TextEditingController controller = TextEditingController(text: 'Flutter');
      addTearDown(controller.dispose);

      await tester.pumpWidget(
        MaterialApp(
          home: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Scaffold(
                body: DropdownMenu<TestMenu>(
                  initialSelection: TestMenu.mainMenu3,
                  // Use a menu entries which does not contain TestMenu.mainMenu3.
                  dropdownMenuEntries: menuChildren.getRange(0, 1).toList(),
                  controller: controller,
                ),
              );
            },
          ),
        ),
      );

      expect(controller.text, 'Flutter');
    },
  );

  testWidgets('The default text input field should not be focused on mobile platforms '
      'when it is tapped', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();

    Widget buildDropdownMenu() => MaterialApp(
      theme: themeData,
      home: Scaffold(
        body: Column(children: <Widget>[DropdownMenu<TestMenu>(dropdownMenuEntries: menuChildren)]),
      ),
    );

    // Test default condition.
    await tester.pumpWidget(buildDropdownMenu());
    await tester.pump();

    final Finder textFieldFinder = find.byType(TextField);
    final TextField result = tester.widget<TextField>(textFieldFinder);
    expect(result.canRequestFocus, false);
  }, variant: TargetPlatformVariant.mobile());

  testWidgets('The text input field should be focused on desktop platforms '
      'when it is tapped', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();

    Widget buildDropdownMenu() => MaterialApp(
      theme: themeData,
      home: Scaffold(
        body: Column(children: <Widget>[DropdownMenu<TestMenu>(dropdownMenuEntries: menuChildren)]),
      ),
    );

    await tester.pumpWidget(buildDropdownMenu());
    await tester.pump();

    final Finder textFieldFinder = find.byType(TextField);
    final TextField result = tester.widget<TextField>(textFieldFinder);
    expect(result.canRequestFocus, true);
  }, variant: TargetPlatformVariant.desktop());

  testWidgets('If requestFocusOnTap is true, the text input field can request focus, '
      'otherwise it cannot request focus', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();

    Widget buildDropdownMenu({required bool requestFocusOnTap}) => MaterialApp(
      theme: themeData,
      home: Scaffold(
        body: Column(
          children: <Widget>[
            DropdownMenu<TestMenu>(
              requestFocusOnTap: requestFocusOnTap,
              dropdownMenuEntries: menuChildren,
            ),
          ],
        ),
      ),
    );

    // Set requestFocusOnTap to true.
    await tester.pumpWidget(buildDropdownMenu(requestFocusOnTap: true));
    await tester.pump();

    final Finder textFieldFinder = find.byType(TextField);
    final TextField textField = tester.widget<TextField>(textFieldFinder);
    expect(textField.canRequestFocus, true);
    // Open the dropdown menu.
    await tester.tap(textFieldFinder);
    await tester.pump();
    // Make a selection.
    await tester.tap(findMenuItemButton('Item 0'));
    await tester.pump();
    expect(findMenuItemButton('Item 0'), findsOneWidget);

    // Set requestFocusOnTap to false.
    await tester.pumpWidget(Container());
    await tester.pumpWidget(buildDropdownMenu(requestFocusOnTap: false));
    await tester.pumpAndSettle();

    final Finder textFieldFinder1 = find.byType(TextField);
    final TextField textField1 = tester.widget<TextField>(textFieldFinder1);
    expect(textField1.canRequestFocus, false);
    // Open the dropdown menu.
    await tester.tap(textFieldFinder1);
    await tester.pump();
    // Make a selection.
    await tester.tap(findMenuItemButton('Item 0'));
    await tester.pump();
    expect(find.widgetWithText(TextField, 'Item 0'), findsOneWidget);
  }, variant: TargetPlatformVariant.all());

  testWidgets('If requestFocusOnTap is false, the mouse cursor should be clickable when hovered', (
    WidgetTester tester,
  ) async {
    Widget buildDropdownMenu() => MaterialApp(
      home: Scaffold(
        body: Column(
          children: <Widget>[
            DropdownMenu<TestMenu>(requestFocusOnTap: false, dropdownMenuEntries: menuChildren),
          ],
        ),
      ),
    );

    await tester.pumpWidget(buildDropdownMenu());
    await tester.pumpAndSettle();

    final Finder textFieldFinder = find.byType(TextField);
    final TextField textField = tester.widget<TextField>(textFieldFinder);
    expect(textField.canRequestFocus, false);

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.moveTo(tester.getCenter(textFieldFinder));
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );
  });

  testWidgets('If enabled is false, the mouse cursor should be deferred when hovered', (
    WidgetTester tester,
  ) async {
    Widget buildDropdownMenu({bool enabled = true, bool? requestFocusOnTap}) {
      return MaterialApp(
        home: Scaffold(
          body: Column(
            children: <Widget>[
              DropdownMenu<TestMenu>(
                enabled: enabled,
                requestFocusOnTap: requestFocusOnTap,
                dropdownMenuEntries: menuChildren,
              ),
            ],
          ),
        ),
      );
    }

    // Check mouse cursor dropdown menu is disabled and requestFocusOnTap is true.
    await tester.pumpWidget(buildDropdownMenu(enabled: false, requestFocusOnTap: true));
    await tester.pumpAndSettle();

    Finder textFieldFinder = find.byType(TextField);
    TextField textField = tester.widget<TextField>(textFieldFinder);
    expect(textField.canRequestFocus, true);

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.moveTo(tester.getCenter(textFieldFinder));
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );

    // Remove the pointer.
    await gesture.removePointer();

    // Check mouse cursor dropdown menu is disabled and requestFocusOnTap is false.
    await tester.pumpWidget(buildDropdownMenu(enabled: false, requestFocusOnTap: false));
    await tester.pumpAndSettle();

    textFieldFinder = find.byType(TextField);
    textField = tester.widget<TextField>(textFieldFinder);
    expect(textField.canRequestFocus, false);

    // Add a new pointer.
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(textFieldFinder));
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );

    // Remove the pointer.
    await gesture.removePointer();

    // Check enabled dropdown menu updates the mouse cursor when hovered.
    await tester.pumpWidget(buildDropdownMenu(requestFocusOnTap: true));
    await tester.pumpAndSettle();

    textFieldFinder = find.byType(TextField);
    textField = tester.widget<TextField>(textFieldFinder);
    expect(textField.canRequestFocus, true);

    // Add a new pointer.
    await gesture.addPointer();
    await gesture.moveTo(tester.getCenter(textFieldFinder));
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );
  });

  testWidgets('The menu has the same width as the input field in ListView', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/123631
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: ListView(
            children: <Widget>[DropdownMenu<TestMenu>(dropdownMenuEntries: menuChildren)],
          ),
        ),
      ),
    );

    final Rect textInput = tester.getRect(find.byType(TextField));

    await tester.tap(find.byType(TextField));
    await tester.pumpAndSettle();

    final Finder findMenu = find.byWidgetPredicate((Widget widget) {
      return widget.runtimeType.toString() == '_MenuPanel';
    });
    final Rect menu = tester.getRect(findMenu);
    expect(textInput.width, menu.width);

    await tester.pumpWidget(Container());
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: ListView(
            children: <Widget>[
              DropdownMenu<TestMenu>(width: 200, dropdownMenuEntries: menuChildren),
            ],
          ),
        ),
      ),
    );

    final Rect textInput1 = tester.getRect(find.byType(TextField));

    await tester.tap(find.byType(TextField));
    await tester.pumpAndSettle();

    final Finder findMenu1 = find.byWidgetPredicate((Widget widget) {
      return widget.runtimeType.toString() == '_MenuPanel';
    });
    final Rect menu1 = tester.getRect(findMenu1);
    expect(textInput1.width, 200);
    expect(menu1.width, 200);
  });

  testWidgets('Semantics does not include hint when input is not empty', (
    WidgetTester tester,
  ) async {
    const String hintText = 'I am hintText';
    TestMenu? selectedValue;
    final TextEditingController controller = TextEditingController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      StatefulBuilder(
        builder:
            (BuildContext context, StateSetter setState) => MaterialApp(
              home: Scaffold(
                body: Center(
                  child: DropdownMenu<TestMenu>(
                    requestFocusOnTap: true,
                    dropdownMenuEntries: menuChildren,
                    hintText: hintText,
                    onSelected: (TestMenu? value) {
                      setState(() {
                        selectedValue = value;
                      });
                    },
                    controller: controller,
                  ),
                ),
              ),
            ),
      ),
    );
    final SemanticsNode node = tester.getSemantics(find.text(hintText));

    expect(selectedValue?.label, null);
    expect(node.label, hintText);
    expect(node.value, '');

    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pumpAndSettle();
    await tester.tap(findMenuItemButton('Item 3'));
    await tester.pumpAndSettle();
    expect(selectedValue?.label, 'Item 3');
    expect(node.label, '');
    expect(node.value, 'Item 3');
  });

  testWidgets('Semantics does not include initial menu buttons', (WidgetTester tester) async {
    final TextEditingController controller = TextEditingController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: DropdownMenu<TestMenu>(
              requestFocusOnTap: true,
              dropdownMenuEntries: menuChildren,
              onSelected: (TestMenu? value) {},
              controller: controller,
            ),
          ),
        ),
      ),
    );
    // The menu buttons should not be visible and should not be in the semantics tree.
    for (final String label in TestMenu.values.map((TestMenu menu) => menu.label)) {
      expect(find.bySemanticsLabel(label), findsNothing);
    }

    // Open the menu.
    await tester.tap(find.widgetWithIcon(IconButton, Icons.arrow_drop_down).first);
    await tester.pump();

    // The menu buttons should be visible and in the semantics tree.
    for (final String label in TestMenu.values.map((TestMenu menu) => menu.label)) {
      expect(find.bySemanticsLabel(label), findsOneWidget);
    }
  });

  testWidgets('helperText is not visible when errorText is not null', (WidgetTester tester) async {
    final ThemeData themeData = ThemeData();
    const String helperText = 'I am helperText';
    const String errorText = 'I am errorText';

    Widget buildFrame(bool hasError) {
      return MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: Center(
            child: DropdownMenu<TestMenu>(
              dropdownMenuEntries: menuChildren,
              helperText: helperText,
              errorText: hasError ? errorText : null,
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(false));
    expect(find.text(helperText), findsOneWidget);
    expect(find.text(errorText), findsNothing);

    await tester.pumpWidget(buildFrame(true));
    await tester.pumpAndSettle();
    expect(find.text(helperText), findsNothing);
    expect(find.text(errorText), findsOneWidget);
  });

  testWidgets('DropdownMenu can respect helperText when helperText is not null', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData();
    const String helperText = 'I am helperText';

    Widget buildFrame() {
      return MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: Center(
            child: DropdownMenu<TestMenu>(
              dropdownMenuEntries: menuChildren,
              helperText: helperText,
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame());
    expect(find.text(helperText), findsOneWidget);
  });

  testWidgets('DropdownMenu can respect errorText when errorText is not null', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData();
    const String errorText = 'I am errorText';

    Widget buildFrame() {
      return MaterialApp(
        theme: themeData,
        home: Scaffold(
          body: Center(
            child: DropdownMenu<TestMenu>(dropdownMenuEntries: menuChildren, errorText: errorText),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame());
    expect(find.text(errorText), findsOneWidget);
  });

  testWidgets('Can scroll to the highlighted item', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenu<TestMenu>(
            requestFocusOnTap: true,
            menuHeight: 100, // Give a small number so the list can only show 2 or 3 items.
            dropdownMenuEntries: menuChildren,
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();

    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pumpAndSettle();

    expect(find.text('Item 5').hitTestable(), findsNothing);
    await tester.enterText(find.byType(TextField), '5');
    await tester.pumpAndSettle();
    // Item 5 should show up.
    expect(find.text('Item 5').hitTestable(), findsOneWidget);
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/131676.
  testWidgets('Material3 - DropdownMenu uses correct text styles', (WidgetTester tester) async {
    const TextStyle inputTextThemeStyle = TextStyle(
      fontSize: 18.5,
      fontStyle: FontStyle.italic,
      wordSpacing: 1.2,
      decoration: TextDecoration.lineThrough,
    );
    const TextStyle menuItemTextThemeStyle = TextStyle(
      fontSize: 20.5,
      fontStyle: FontStyle.italic,
      wordSpacing: 2.1,
      decoration: TextDecoration.underline,
    );
    final ThemeData themeData = ThemeData(
      textTheme: const TextTheme(
        bodyLarge: inputTextThemeStyle,
        labelLarge: menuItemTextThemeStyle,
      ),
    );
    await tester.pumpWidget(buildTest(themeData, menuChildren));

    // Test input text style uses the TextTheme.bodyLarge.
    final EditableText editableText = tester.widget(find.byType(EditableText));
    expect(editableText.style.fontSize, inputTextThemeStyle.fontSize);
    expect(editableText.style.fontStyle, inputTextThemeStyle.fontStyle);
    expect(editableText.style.wordSpacing, inputTextThemeStyle.wordSpacing);
    expect(editableText.style.decoration, inputTextThemeStyle.decoration);

    // Open the menu.
    await tester.tap(find.widgetWithIcon(IconButton, Icons.arrow_drop_down).first);
    await tester.pump();

    // Test menu item text style uses the TextTheme.labelLarge.
    final Material material = getButtonMaterial(tester, TestMenu.mainMenu0.label);
    expect(material.textStyle?.fontSize, menuItemTextThemeStyle.fontSize);
    expect(material.textStyle?.fontStyle, menuItemTextThemeStyle.fontStyle);
    expect(material.textStyle?.wordSpacing, menuItemTextThemeStyle.wordSpacing);
    expect(material.textStyle?.decoration, menuItemTextThemeStyle.decoration);
  });

  testWidgets('DropdownMenuEntries do not overflow when width is specified', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/126882
    final TextEditingController controller = TextEditingController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenu<TestMenu>(
            controller: controller,
            width: 100,
            dropdownMenuEntries:
                TestMenu.values.map<DropdownMenuEntry<TestMenu>>((TestMenu item) {
                  return DropdownMenuEntry<TestMenu>(value: item, label: '${item.label} $longText');
                }).toList(),
          ),
        ),
      ),
    );

    // Opening the width=100 menu should not crash.
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    expect(tester.takeException(), isNull);
    await tester.pumpAndSettle();

    Finder findMenuItemText(String label) {
      final String labelText = '$label $longText';
      return find.descendant(of: findMenuItemButton(labelText), matching: find.byType(Text)).last;
    }

    // Actual size varies a little on web platforms.
    final Matcher closeTo300 = closeTo(300, 0.25);
    expect(tester.getSize(findMenuItemText('Item 0')).height, closeTo300);
    expect(tester.getSize(findMenuItemText('Menu 1')).height, closeTo300);
    expect(tester.getSize(findMenuItemText('Item 2')).height, closeTo300);
    expect(tester.getSize(findMenuItemText('Item 3')).height, closeTo300);

    await tester.tap(findMenuItemText('Item 0'));
    await tester.pumpAndSettle();
    expect(controller.text, 'Item 0 $longText');
  });

  testWidgets('DropdownMenuEntry.labelWidget is Text that specifies maxLines 1 or 2', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/126882
    final TextEditingController controller = TextEditingController();
    addTearDown(controller.dispose);

    Widget buildFrame({required int maxLines}) {
      return MaterialApp(
        home: Scaffold(
          body: DropdownMenu<TestMenu>(
            key: ValueKey<int>(maxLines),
            controller: controller,
            width: 100,
            dropdownMenuEntries:
                TestMenu.values.map<DropdownMenuEntry<TestMenu>>((TestMenu item) {
                  return DropdownMenuEntry<TestMenu>(
                    value: item,
                    label: '${item.label} $longText',
                    labelWidget: Text('${item.label} $longText', maxLines: maxLines),
                  );
                }).toList(),
          ),
        ),
      );
    }

    Finder findMenuItemText(String label) {
      final String labelText = '$label $longText';
      return find.descendant(of: findMenuItemButton(labelText), matching: find.byType(Text)).last;
    }

    await tester.pumpWidget(buildFrame(maxLines: 1));
    await tester.tap(find.byType(DropdownMenu<TestMenu>));

    // Actual size varies a little on web platforms.
    final Matcher closeTo20 = closeTo(20, 0.05);
    expect(tester.getSize(findMenuItemText('Item 0')).height, closeTo20);
    expect(tester.getSize(findMenuItemText('Menu 1')).height, closeTo20);
    expect(tester.getSize(findMenuItemText('Item 2')).height, closeTo20);
    expect(tester.getSize(findMenuItemText('Item 3')).height, closeTo20);

    // Close the menu
    await tester.tap(find.byType(TextField));
    await tester.pumpAndSettle();
    expect(controller.text, ''); // nothing selected

    await tester.pumpWidget(buildFrame(maxLines: 2));
    await tester.tap(find.byType(DropdownMenu<TestMenu>));

    // Actual size varies a little on web platforms.
    final Matcher closeTo40 = closeTo(40, 0.05);
    expect(tester.getSize(findMenuItemText('Item 0')).height, closeTo40);
    expect(tester.getSize(findMenuItemText('Menu 1')).height, closeTo40);
    expect(tester.getSize(findMenuItemText('Item 2')).height, closeTo40);
    expect(tester.getSize(findMenuItemText('Item 3')).height, closeTo40);

    // Close the menu
    await tester.tap(find.byType(TextField));
    await tester.pumpAndSettle();
    expect(controller.text, ''); // nothing selected
  });

  // Regression test for https://github.com/flutter/flutter/issues/131350.
  testWidgets('DropdownMenuEntry.leadingIcon default layout', (WidgetTester tester) async {
    // The DropdownMenu should not get extra padding in DropdownMenuEntry items
    // when both text field and DropdownMenuEntry have leading icons.
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: DropdownMenu<int>(
            leadingIcon: Icon(Icons.search),
            hintText: 'Hint',
            dropdownMenuEntries: <DropdownMenuEntry<int>>[
              DropdownMenuEntry<int>(value: 0, label: 'Item 0', leadingIcon: Icon(Icons.alarm)),
              DropdownMenuEntry<int>(value: 1, label: 'Item 1'),
            ],
          ),
        ),
      ),
    );
    await tester.tap(find.byType(DropdownMenu<int>));
    await tester.pumpAndSettle();

    // Check text location in text field.
    expect(tester.getTopLeft(find.text('Hint')).dx, 52.0);

    // By default, the text of item 0 should be aligned with the text of the text field.
    expect(tester.getTopLeft(find.text('Item 0').last).dx, 52.0);

    // By default, the text of item 1 should be aligned with the text of the text field,
    // so there are some extra padding before "Item 1".
    expect(tester.getTopLeft(find.text('Item 1').last).dx, 52.0);
  });

  testWidgets('DropdownMenu can have customized search algorithm', (WidgetTester tester) async {
    final ThemeData theme = ThemeData();
    Widget dropdownMenu({SearchCallback<int>? searchCallback}) {
      return MaterialApp(
        theme: theme,
        home: Scaffold(
          body: DropdownMenu<int>(
            requestFocusOnTap: true,
            searchCallback: searchCallback,
            dropdownMenuEntries: const <DropdownMenuEntry<int>>[
              DropdownMenuEntry<int>(value: 0, label: 'All'),
              DropdownMenuEntry<int>(value: 1, label: 'Unread'),
              DropdownMenuEntry<int>(value: 2, label: 'Read'),
            ],
          ),
        ),
      );
    }

    void checkExpectedHighlight({String? searchResult, required List<String> otherItems}) {
      if (searchResult != null) {
        final Finder material = find.descendant(
          of: findMenuItemButton(searchResult),
          matching: find.byType(Material),
        );
        final Material itemMaterial = tester.widget<Material>(material);
        expect(itemMaterial.color, theme.colorScheme.onSurface.withOpacity(0.12));
      }

      for (final String nonHighlight in otherItems) {
        final Finder material = find.descendant(
          of: findMenuItemButton(nonHighlight),
          matching: find.byType(Material),
        );
        final Material itemMaterial = tester.widget<Material>(material);
        expect(itemMaterial.color, Colors.transparent);
      }
    }

    // Test default.
    await tester.pumpWidget(dropdownMenu());
    await tester.pump();
    await tester.tap(find.byType(DropdownMenu<int>));
    await tester.pumpAndSettle();

    await tester.enterText(find.byType(TextField), 'read');
    await tester.pump();
    checkExpectedHighlight(
      searchResult: 'Unread',
      otherItems: <String>['All', 'Read'],
    ); // Because "Unread" contains "read".

    // Test custom search algorithm.
    await tester.pumpWidget(dropdownMenu(searchCallback: (_, __) => 0));
    await tester.pump();
    await tester.enterText(find.byType(TextField), 'read');
    await tester.pump();
    checkExpectedHighlight(
      searchResult: 'All',
      otherItems: <String>['Unread', 'Read'],
    ); // Because the search result should always be index 0.

    // Test custom search algorithm - exact match.
    await tester.pumpWidget(
      dropdownMenu(
        searchCallback: (List<DropdownMenuEntry<int>> entries, String query) {
          if (query.isEmpty) {
            return null;
          }
          final int index = entries.indexWhere(
            (DropdownMenuEntry<int> entry) => entry.label == query,
          );

          return index != -1 ? index : null;
        },
      ),
    );
    await tester.pump();

    await tester.enterText(find.byType(TextField), 'read');
    await tester.pump();
    checkExpectedHighlight(
      otherItems: <String>['All', 'Unread', 'Read'],
    ); // Because it's case sensitive.
    await tester.enterText(find.byType(TextField), 'Read');
    await tester.pump();
    checkExpectedHighlight(searchResult: 'Read', otherItems: <String>['All', 'Unread']);
  });

  testWidgets('onSelected gets called when a selection is made in a nested menu', (
    WidgetTester tester,
  ) async {
    int selectionCount = 0;

    final ThemeData themeData = ThemeData();
    final List<DropdownMenuEntry<TestMenu>> menuWithDisabledItems = <DropdownMenuEntry<TestMenu>>[
      const DropdownMenuEntry<TestMenu>(value: TestMenu.mainMenu0, label: 'Item 0'),
    ];

    await tester.pumpWidget(
      MaterialApp(
        theme: themeData,
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Scaffold(
              body: MenuAnchor(
                menuChildren: <Widget>[
                  DropdownMenu<TestMenu>(
                    dropdownMenuEntries: menuWithDisabledItems,
                    onSelected: (_) {
                      setState(() {
                        selectionCount++;
                      });
                    },
                  ),
                ],
                builder: (BuildContext context, MenuController controller, Widget? widget) {
                  return IconButton(
                    icon: const Icon(Icons.smartphone_rounded),
                    onPressed: () {
                      controller.open();
                    },
                  );
                },
              ),
            );
          },
        ),
      ),
    );

    // Open the first menu
    await tester.tap(find.byType(IconButton));
    await tester.pump();
    // Open the dropdown menu
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pump();

    final Finder item1 = findMenuItemButton('Item 0');
    await tester.tap(item1);
    await tester.pumpAndSettle();

    expect(selectionCount, 1);
  });

  testWidgets(
    'When onSelected is called and menu is closed, no textEditingController exception is thrown',
    (WidgetTester tester) async {
      int selectionCount = 0;

      final ThemeData themeData = ThemeData();
      final List<DropdownMenuEntry<TestMenu>> menuWithDisabledItems = <DropdownMenuEntry<TestMenu>>[
        const DropdownMenuEntry<TestMenu>(value: TestMenu.mainMenu0, label: 'Item 0'),
      ];

      await tester.pumpWidget(
        MaterialApp(
          theme: themeData,
          home: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              return Scaffold(
                body: MenuAnchor(
                  menuChildren: <Widget>[
                    DropdownMenu<TestMenu>(
                      dropdownMenuEntries: menuWithDisabledItems,
                      onSelected: (_) {
                        setState(() {
                          selectionCount++;
                        });
                      },
                    ),
                  ],
                  builder: (BuildContext context, MenuController controller, Widget? widget) {
                    return IconButton(
                      icon: const Icon(Icons.smartphone_rounded),
                      onPressed: () {
                        controller.open();
                      },
                    );
                  },
                ),
              );
            },
          ),
        ),
      );

      // Open the first menu
      await tester.tap(find.byType(IconButton));
      await tester.pump();
      // Open the dropdown menu
      await tester.tap(find.byType(DropdownMenu<TestMenu>));
      await tester.pump();

      final Finder item1 = findMenuItemButton('Item 0');
      await tester.tap(item1);
      await tester.pumpAndSettle();

      expect(selectionCount, 1);
      expect(tester.takeException(), isNull);
    },
  );

  // Regression test for https://github.com/flutter/flutter/issues/139871.
  testWidgets(
    'setState is not called through addPostFrameCallback after DropdownMenu is unmounted',
    (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: ListView.builder(
              itemCount: 500,
              itemBuilder: (BuildContext context, int index) {
                if (index == 250) {
                  return DropdownMenu<TestMenu>(dropdownMenuEntries: menuChildren);
                } else {
                  return Container(height: 50);
                }
              },
            ),
          ),
        ),
      );

      await tester.fling(find.byType(ListView), const Offset(0, -20000), 200000.0);

      await tester.pumpAndSettle();

      expect(tester.takeException(), isNull);
    },
  );

  testWidgets('Menu shows scrollbar when height is limited', (WidgetTester tester) async {
    final List<DropdownMenuEntry<TestMenu>> menuItems = <DropdownMenuEntry<TestMenu>>[
      DropdownMenuEntry<TestMenu>(
        value: TestMenu.mainMenu0,
        label: 'Item 0',
        style: MenuItemButton.styleFrom(minimumSize: const Size.fromHeight(1000)),
      ),
    ];

    await tester.pumpWidget(
      MaterialApp(home: Scaffold(body: DropdownMenu<TestMenu>(dropdownMenuEntries: menuItems))),
    );

    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pumpAndSettle();

    expect(find.byType(Scrollbar), findsOneWidget);
  }, variant: TargetPlatformVariant.all());

  testWidgets('DropdownMenu.focusNode can focus text input field', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);
    final ThemeData theme = ThemeData();

    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: Scaffold(
          body: DropdownMenu<String>(
            focusNode: focusNode,
            dropdownMenuEntries: const <DropdownMenuEntry<String>>[
              DropdownMenuEntry<String>(value: 'Yolk', label: 'Yolk'),
              DropdownMenuEntry<String>(value: 'Eggbert', label: 'Eggbert'),
            ],
          ),
        ),
      ),
    );

    RenderBox box = tester.renderObject(find.byType(InputDecorator));

    // Test input border when not focused.
    expect(box, paints..rrect(color: theme.colorScheme.outline));

    focusNode.requestFocus();
    await tester.pump();
    // Advance input decorator animation.
    await tester.pump(const Duration(milliseconds: 200));

    box = tester.renderObject(find.byType(InputDecorator));

    // Test input border when focused.
    expect(box, paints..rrect(color: theme.colorScheme.primary));
  });

  // Regression test for https://github.com/flutter/flutter/issues/131120.
  testWidgets('Focus traversal ignores non visible entries', (WidgetTester tester) async {
    final FocusNode buttonFocusNode = FocusNode();
    addTearDown(buttonFocusNode.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Column(
            children: <Widget>[
              DropdownMenu<TestMenu>(dropdownMenuEntries: menuChildren),
              ElevatedButton(
                focusNode: buttonFocusNode,
                onPressed: () {},
                child: const Text('Button'),
              ),
            ],
          ),
        ),
      ),
    );

    // Move the focus to the text field.
    primaryFocus!.nextFocus();
    await tester.pump();
    final Element textField = tester.element(find.byType(TextField));
    expect(Focus.of(textField).hasFocus, isTrue);

    // Move the focus to the dropdown trailing icon.
    primaryFocus!.nextFocus();
    await tester.pump();
    final Element iconButton = tester.firstElement(find.byIcon(Icons.arrow_drop_down));
    expect(Focus.of(iconButton).hasFocus, isTrue);

    // Move the focus to the elevated button.
    primaryFocus!.nextFocus();
    await tester.pump();
    expect(buttonFocusNode.hasFocus, isTrue);
  });

  testWidgets('DropdownMenu honors inputFormatters', (WidgetTester tester) async {
    int called = 0;
    final TextInputFormatter formatter = TextInputFormatter.withFunction((
      TextEditingValue oldValue,
      TextEditingValue newValue,
    ) {
      called += 1;
      return newValue;
    });
    final TextEditingController controller = TextEditingController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenu<String>(
            requestFocusOnTap: true,
            controller: controller,
            dropdownMenuEntries: const <DropdownMenuEntry<String>>[
              DropdownMenuEntry<String>(value: 'Blue', label: 'Blue'),
              DropdownMenuEntry<String>(value: 'Green', label: 'Green'),
            ],
            inputFormatters: <TextInputFormatter>[
              formatter,
              FilteringTextInputFormatter.deny(RegExp('[0-9]')),
            ],
          ),
        ),
      ),
    );

    final EditableTextState state = tester.firstState(find.byType(EditableText));
    state.updateEditingValue(const TextEditingValue(text: 'Blue'));
    expect(called, 1);
    expect(controller.text, 'Blue');

    state.updateEditingValue(const TextEditingValue(text: 'Green'));
    expect(called, 2);
    expect(controller.text, 'Green');

    state.updateEditingValue(const TextEditingValue(text: 'Green2'));
    expect(called, 3);
    expect(controller.text, 'Green');
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/140596.
  testWidgets('Long text item does not overflow', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenu<int>(
            dropdownMenuEntries: <DropdownMenuEntry<int>>[
              DropdownMenuEntry<int>(
                value: 0,
                label: 'This is a long text that is multiplied by 4 so it can overflow. ' * 4,
              ),
            ],
          ),
        ),
      ),
    );

    await tester.pump();
    await tester.tap(find.byType(DropdownMenu<int>));
    await tester.pumpAndSettle();

    // No exception should be thrown.
    expect(tester.takeException(), isNull);
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/147076.
  testWidgets('Text field does not overflow parent', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: SizedBox(
            width: 300,
            child: DropdownMenu<int>(
              dropdownMenuEntries: <DropdownMenuEntry<int>>[
                DropdownMenuEntry<int>(
                  value: 0,
                  label: 'This is a long text that is multiplied by 4 so it can overflow. ' * 4,
                ),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.pump();
    final RenderBox box = tester.firstRenderObject(find.byType(TextField));
    expect(box.size.width, 300.0);
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/147173.
  testWidgets('Text field with large helper text can be selected', (WidgetTester tester) async {
    const String labelText = 'MenuEntry 1';
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: Center(
            child: DropdownMenu<int>(
              hintText: 'Hint text',
              helperText: 'Menu Helper text',
              inputDecorationTheme: InputDecorationTheme(
                helperMaxLines: 2,
                helperStyle: TextStyle(fontSize: 30),
              ),
              dropdownMenuEntries: <DropdownMenuEntry<int>>[
                DropdownMenuEntry<int>(value: 0, label: labelText),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.pump();
    await tester.tapAt(tester.getCenter(find.text('Hint text')));
    await tester.pumpAndSettle();
    // One is layout for the _DropdownMenuBody, the other one is the real button item in the menu.
    expect(find.widgetWithText(MenuItemButton, labelText), findsNWidgets(2));
  });

  testWidgets('DropdownMenu allows customizing text field text align', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: Column(
            children: <DropdownMenu<int>>[
              DropdownMenu<int>(dropdownMenuEntries: <DropdownMenuEntry<int>>[]),
              DropdownMenu<int>(
                textAlign: TextAlign.center,
                dropdownMenuEntries: <DropdownMenuEntry<int>>[],
              ),
            ],
          ),
        ),
      ),
    );

    final List<TextField> fields = tester.widgetList<TextField>(find.byType(TextField)).toList();

    expect(fields[0].textAlign, TextAlign.start);
    expect(fields[1].textAlign, TextAlign.center);
  });

  testWidgets('DropdownMenu correctly sets keyboardType on TextField', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: SafeArea(
            child: DropdownMenu<TestMenu>(
              dropdownMenuEntries: menuChildren,
              keyboardType: TextInputType.number,
            ),
          ),
        ),
      ),
    );

    final TextField textField = tester.widget(find.byType(TextField));
    expect(textField.keyboardType, TextInputType.number);
  });

  testWidgets('DropdownMenu keyboardType defaults to TextInputType.text', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: SafeArea(child: DropdownMenu<TestMenu>(dropdownMenuEntries: menuChildren)),
        ),
      ),
    );

    final TextField textField = tester.widget(find.byType(TextField));
    expect(textField.keyboardType, TextInputType.text);
  });

  testWidgets('DropdownMenu passes an alignmentOffset to MenuAnchor', (WidgetTester tester) async {
    const Offset alignmentOffset = Offset(0, 16);

    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: DropdownMenu<String>(
            alignmentOffset: alignmentOffset,
            dropdownMenuEntries: <DropdownMenuEntry<String>>[
              DropdownMenuEntry<String>(value: '1', label: 'One'),
              DropdownMenuEntry<String>(value: '2', label: 'Two'),
            ],
          ),
        ),
      ),
    );

    final MenuAnchor menuAnchor = tester.widget<MenuAnchor>(find.byType(MenuAnchor));

    expect(menuAnchor.alignmentOffset, alignmentOffset);
  });

  testWidgets('DropdownMenu filter is disabled until text input', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenu<TestMenu>(
            requestFocusOnTap: true,
            enableFilter: true,
            initialSelection: menuChildren[0].value,
            dropdownMenuEntries: menuChildren,
          ),
        ),
      ),
    );

    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pumpAndSettle();

    // All entries should be available, and two buttons should be found for each entry.
    // One is layout for the _DropdownMenuBody, the other one is the real button item in the menu.
    for (final TestMenu menu in TestMenu.values) {
      expect(find.widgetWithText(MenuItemButton, menu.label), findsNWidgets(2));
    }

    // Text input would enable the filter.
    await tester.enterText(find.byType(TextField).first, 'Menu 1');
    await tester.pumpAndSettle();
    for (final TestMenu menu in TestMenu.values) {
      // 'Menu 1' should be 2, other items should only find one.
      if (menu.label == TestMenu.mainMenu1.label) {
        expect(find.widgetWithText(MenuItemButton, menu.label), findsNWidgets(2));
      } else {
        expect(find.widgetWithText(MenuItemButton, menu.label), findsOneWidget);
      }
    }

    // Selecting an item would disable filter again.
    await tester.tap(findMenuItemButton('Menu 1'));
    await tester.pumpAndSettle();
    await tester.tap(find.byType(DropdownMenu<TestMenu>));
    await tester.pumpAndSettle();
    for (final TestMenu menu in TestMenu.values) {
      expect(find.widgetWithText(MenuItemButton, menu.label), findsNWidgets(2));
    }
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/151686.
  testWidgets('Setting DropdownMenu.requestFocusOnTap to false makes TextField read only', (
    WidgetTester tester,
  ) async {
    const String label = 'Test';
    Widget buildDropdownMenu({bool? requestFocusOnTap}) {
      return MaterialApp(
        home: Scaffold(
          body: Center(
            child: DropdownMenu<TestMenu>(
              requestFocusOnTap: requestFocusOnTap,
              dropdownMenuEntries: menuChildren,
              hintText: label,
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildDropdownMenu(requestFocusOnTap: true));

    expect(
      tester.getSemantics(find.byType(TextField)),
      matchesSemantics(
        hasFocusAction: true,
        hasTapAction: true,
        isTextField: true,
        hasEnabledState: true,
        isEnabled: true,
        label: 'Test',
        textDirection: TextDirection.ltr,
      ),
    );

    await tester.pumpWidget(buildDropdownMenu(requestFocusOnTap: false));

    expect(
      tester.getSemantics(find.byType(TextField)),
      matchesSemantics(
        hasFocusAction: true,
        isTextField: true,
        hasEnabledState: true,
        isEnabled: true,
        label: 'Test',
        isReadOnly: true,
        textDirection: TextDirection.ltr,
      ),
    );
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/151854.
  testWidgets('scrollToHighlight does not scroll parent', (WidgetTester tester) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: ListView(
            controller: controller,
            children: <Widget>[
              ListView(
                shrinkWrap: true,
                children: <Widget>[
                  DropdownMenu<TestMenu>(
                    initialSelection: menuChildren.last.value,
                    dropdownMenuEntries: menuChildren,
                  ),
                ],
              ),
              const SizedBox(height: 1000.0),
            ],
          ),
        ),
      ),
    );

    await tester.tap(find.byType(TextField).first);
    await tester.pumpAndSettle();
    expect(controller.offset, 0.0);
  });

  testWidgets('DropdownMenu with expandedInsets can be aligned', (WidgetTester tester) async {
    Widget buildMenuAnchor({AlignmentGeometry alignment = Alignment.topCenter}) {
      return MaterialApp(
        home: Scaffold(
          body: Row(
            children: <Widget>[
              Expanded(
                child: Align(
                  alignment: alignment,
                  child: DropdownMenu<TestMenu>(
                    expandedInsets: const EdgeInsets.all(16),
                    dropdownMenuEntries: menuChildren,
                  ),
                ),
              ),
            ],
          ),
        ),
      );
    }

    await tester.pumpWidget(buildMenuAnchor());

    Offset textFieldPosition = tester.getTopLeft(find.byType(TextField));
    expect(textFieldPosition, equals(const Offset(16.0, 0.0)));

    await tester.pumpWidget(buildMenuAnchor(alignment: Alignment.center));

    textFieldPosition = tester.getTopLeft(find.byType(TextField));
    expect(textFieldPosition, equals(const Offset(16.0, 272.0)));

    await tester.pumpWidget(buildMenuAnchor(alignment: Alignment.bottomCenter));

    textFieldPosition = tester.getTopLeft(find.byType(TextField));
    expect(textFieldPosition, equals(const Offset(16.0, 544.0)));
  });

  // Regression test for https://github.com/flutter/flutter/issues/139269.
  testWidgets('DropdownMenu.closeBehavior controls menu closing behavior', (
    WidgetTester tester,
  ) async {
    Widget buildDropdownMenu({
      DropdownMenuCloseBehavior closeBehavior = DropdownMenuCloseBehavior.all,
    }) {
      return MaterialApp(
        home: Scaffold(
          body: MenuAnchor(
            menuChildren: <Widget>[
              DropdownMenu<TestMenu>(
                closeBehavior: closeBehavior,
                dropdownMenuEntries: menuChildren,
              ),
            ],
            child: const Text('Open Menu'),
            builder: (BuildContext context, MenuController controller, Widget? child) {
              return ElevatedButton(onPressed: () => controller.open(), child: child);
            },
          ),
        ),
      );
    }

    // Test closeBehavior set to all.
    await tester.pumpWidget(buildDropdownMenu());

    // Tap the button to open the root anchor.
    await tester.tap(find.byType(ElevatedButton));
    await tester.pumpAndSettle();
    // Tap the menu item to open the dropdown menu.
    await tester.tap(find.byType(TextField));
    await tester.pumpAndSettle();
    expect(find.byType(DropdownMenu<TestMenu>), findsOneWidget);

    MenuAnchor dropdownMenuAnchor = tester.widget<MenuAnchor>(find.byType(MenuAnchor).last);
    expect(dropdownMenuAnchor.controller!.isOpen, true);

    // Tap the dropdown menu item.
    await tester.tap(findMenuItemButton(TestMenu.mainMenu0.label));
    await tester.pumpAndSettle();
    // All menus should be closed.
    expect(find.byType(DropdownMenu<TestMenu>), findsNothing);
    expect(find.byType(MenuAnchor), findsOneWidget);

    // Test closeBehavior set to self.
    await tester.pumpWidget(buildDropdownMenu(closeBehavior: DropdownMenuCloseBehavior.self));

    // Tap the button to open the root anchor.
    await tester.tap(find.byType(ElevatedButton));
    await tester.pumpAndSettle();
    expect(find.byType(DropdownMenu<TestMenu>), findsOneWidget);

    // Tap the menu item to open the dropdown menu.
    await tester.tap(find.byType(TextField));
    await tester.pumpAndSettle();
    dropdownMenuAnchor = tester.widget<MenuAnchor>(find.byType(MenuAnchor).last);
    expect(dropdownMenuAnchor.controller!.isOpen, true);

    // Tap the menu item to open the dropdown menu.
    await tester.tap(findMenuItemButton(TestMenu.mainMenu0.label));
    await tester.pumpAndSettle();
    // Only the dropdown menu should be closed.
    expect(dropdownMenuAnchor.controller!.isOpen, false);

    // Test closeBehavior set to none.
    await tester.pumpWidget(buildDropdownMenu(closeBehavior: DropdownMenuCloseBehavior.none));

    // Tap the button to open the root anchor.
    await tester.tap(find.byType(ElevatedButton));
    await tester.pumpAndSettle();
    expect(find.byType(DropdownMenu<TestMenu>), findsOneWidget);

    // Tap the menu item to open the dropdown menu.
    await tester.tap(find.byType(TextField));
    await tester.pumpAndSettle();
    dropdownMenuAnchor = tester.widget<MenuAnchor>(find.byType(MenuAnchor).last);
    expect(dropdownMenuAnchor.controller!.isOpen, true);

    // Tap the dropdown menu item.
    await tester.tap(findMenuItemButton(TestMenu.mainMenu0.label));
    await tester.pumpAndSettle();
    // None of the menus should be closed.
    expect(dropdownMenuAnchor.controller!.isOpen, true);
  });

  group('The menu is attached at the bottom of the TextField', () {
    // Define the expected text field bottom instead of querying it using
    // tester.getRect because when tight constraints are applied to the
    // Dropdown the TextField bounds are expanded while the visible size
    // remains 56 pixels.
    const double textFieldBottom = 56.0;

    testWidgets('when given loose constraints and expandedInsets is set', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: DropdownMenu<TestMenu>(
              expandedInsets: EdgeInsets.zero,
              initialSelection: TestMenu.mainMenu3,
              dropdownMenuEntries: menuChildrenWithIcons,
            ),
          ),
        ),
      );

      // Open the menu.
      await tester.tap(find.byType(TextField));
      await tester.pump();

      expect(tester.getRect(findMenuMaterial()).top, textFieldBottom);
    });

    testWidgets('when given tight constraints and expandedInsets is set', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: SizedBox(
              width: 200,
              height: 300,
              child: DropdownMenu<TestMenu>(
                expandedInsets: EdgeInsets.zero,
                initialSelection: TestMenu.mainMenu3,
                dropdownMenuEntries: menuChildrenWithIcons,
              ),
            ),
          ),
        ),
      );

      // Open the menu.
      await tester.tap(find.byType(TextField));
      await tester.pump();

      expect(tester.getRect(findMenuMaterial()).top, textFieldBottom);
    });

    // Regression test for https://github.com/flutter/flutter/issues/147076.
    testWidgets('when given loose constraints and expandedInsets is not set', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: DropdownMenu<TestMenu>(
              initialSelection: TestMenu.mainMenu3,
              dropdownMenuEntries: menuChildrenWithIcons,
            ),
          ),
        ),
      );

      // Open the menu.
      await tester.tap(find.byType(TextField));
      await tester.pump();

      expect(tester.getRect(findMenuMaterial()).top, textFieldBottom);
    });

    // Regression test for https://github.com/flutter/flutter/issues/147076.
    testWidgets('when given tight constraints and expandedInsets is not set', (
      WidgetTester tester,
    ) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: SizedBox(
              width: 200,
              height: 300,
              child: DropdownMenu<TestMenu>(
                initialSelection: TestMenu.mainMenu3,
                dropdownMenuEntries: menuChildrenWithIcons,
              ),
            ),
          ),
        ),
      );

      // Open the menu.
      await tester.tap(find.byType(TextField));
      await tester.pump();

      expect(tester.getRect(findMenuMaterial()).top, textFieldBottom);
    });
  });

  // Regression test for https://github.com/flutter/flutter/issues/143505.
  testWidgets('Using keyboard navigation to select', (WidgetTester tester) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);
    TestMenu? selectedMenu;
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: DropdownMenu<TestMenu>(
              focusNode: focusNode,
              dropdownMenuEntries: menuChildren,
              onSelected: (TestMenu? menu) {
                selectedMenu = menu;
              },
            ),
          ),
        ),
      ),
    );
    // Pressing the tab key 3 times moves the focus to the icon button.
    for (int i = 0; i < 3; i++) {
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pump();
    }

    // Now the focus is on the icon button.
    final Element iconButton = tester.firstElement(find.byIcon(Icons.arrow_drop_down));
    expect(Focus.of(iconButton).hasPrimaryFocus, isTrue);

    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pump();

    await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
    await tester.pump();

    await tester.sendKeyEvent(LogicalKeyboardKey.enter);
    await tester.pump();

    expect(selectedMenu, TestMenu.mainMenu0);
  }, variant: TargetPlatformVariant.all());

  // Regression test for https://github.com/flutter/flutter/issues/143505.
  testWidgets(
    'Using keyboard navigation to select and without setting the FocusNode parameter',
    (WidgetTester tester) async {
      TestMenu? selectedMenu;
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: DropdownMenu<TestMenu>(
                dropdownMenuEntries: menuChildren,
                onSelected: (TestMenu? menu) {
                  selectedMenu = menu;
                },
              ),
            ),
          ),
        ),
      );
      // If there is no `FocusNode`, by default, `TextField` can receive focus
      // on desktop platforms, but not on mobile platforms. Therefore, on desktop
      // platforms, it takes 3 tabs to reach the icon button.
      final int tabCount = switch (defaultTargetPlatform) {
        TargetPlatform.iOS || TargetPlatform.android || TargetPlatform.fuchsia => 2,
        TargetPlatform.macOS || TargetPlatform.linux || TargetPlatform.windows => 3,
      };
      for (int i = 0; i < tabCount; i++) {
        await tester.sendKeyEvent(LogicalKeyboardKey.tab);
        await tester.pump();
      }

      // Now the focus is on the icon button.
      final Element iconButton = tester.firstElement(find.byIcon(Icons.arrow_drop_down));
      expect(Focus.of(iconButton).hasPrimaryFocus, isTrue);

      await tester.sendKeyEvent(LogicalKeyboardKey.enter);
      await tester.pump();

      await tester.sendKeyEvent(LogicalKeyboardKey.arrowDown);
      await tester.pump();

      await tester.sendKeyEvent(LogicalKeyboardKey.enter);
      await tester.pump();

      expect(selectedMenu, TestMenu.mainMenu0);
    },
    variant: TargetPlatformVariant.all(),
  );
}

enum TestMenu {
  mainMenu0('Item 0'),
  mainMenu1('Menu 1'),
  mainMenu2('Item 2'),
  mainMenu3('Item 3'),
  mainMenu4('Item 4'),
  mainMenu5('Item 5');

  const TestMenu(this.label);
  final String label;
}

enum ShortMenu {
  item0('I0'),
  item1('I1'),
  item2('I2');

  const ShortMenu(this.label);
  final String label;
}
