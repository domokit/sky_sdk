// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/src/services/text_formatter.dart';
import 'package:flutter_test/flutter_test.dart';

enum MenuItem {
  menuItem0('Item 0'),
  menuItem1('Item 1'),
  menuItem2('Item 2'),
  menuItem3('Item 3');

  const MenuItem(this.label);
  final String label;
}

void main() {
  final List<DropdownMenuEntry<MenuItem>> menuEntries = <DropdownMenuEntry<MenuItem>>[];

  for (final MenuItem value in MenuItem.values) {
    final DropdownMenuEntry<MenuItem> entry = DropdownMenuEntry<MenuItem>(
      value: value,
      label: value.label,
    );
    menuEntries.add(entry);
  }

  Finder findMenuItem(MenuItem menuItem) {
    // For each menu items there are two MenuItemButton widgets.
    // The last one is the real button item in the menu.
    // The first one is not visible, it is part of _DropdownMenuBody
    // which is used to compute the dropdown width.
    return find.widgetWithText(MenuItemButton, menuItem.label).last;
  }

  testWidgets('Creates an underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    expect(find.byType(DropdownMenu<MenuItem>), findsOne);
  });

  testWidgets('Passes dropdownMenuEntries to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    final DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.dropdownMenuEntries, menuEntries);
  });

  testWidgets('Dropdown menu can be opened and contains all the items', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    await tester.tap(find.byType(DropdownMenu<MenuItem>));
    await tester.pump();

    for (final MenuItem item in MenuItem.values) {
      expect(findMenuItem(item), findsOne);
    }
  });

  testWidgets('Passes enabled to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.enabled, true);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(enabled: false, dropdownMenuEntries: menuEntries),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.enabled, false);
  });

  testWidgets('Passes width to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.width, null);

    const double width = 100.0;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(width: width, dropdownMenuEntries: menuEntries),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.width, width);
  });

  testWidgets('Passes menuHeight to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.menuHeight, null);

    const double menuHeight = 100.0;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            menuHeight: menuHeight,
            dropdownMenuEntries: menuEntries,
          ),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.menuHeight, menuHeight);
  });

  testWidgets('Passes leadingIcon to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.leadingIcon, null);

    const Icon leadingIcon = Icon(Icons.abc);
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            leadingIcon: leadingIcon,
            dropdownMenuEntries: menuEntries,
          ),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.leadingIcon, leadingIcon);
  });

  testWidgets('Passes trailingIcon to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.trailingIcon, null);

    const Icon trailingIcon = Icon(Icons.abc);
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            trailingIcon: trailingIcon,
            dropdownMenuEntries: menuEntries,
          ),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.trailingIcon, trailingIcon);
  });

  testWidgets('Passes label to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.label, null);

    const Widget label = Text('Label');
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(label: label, dropdownMenuEntries: menuEntries),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.label, label);
  });

  testWidgets('Passes hintText to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.hintText, null);

    const String hintText = 'Hint';
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            hintText: hintText,
            dropdownMenuEntries: menuEntries,
          ),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.hintText, hintText);
  });

  testWidgets('Passes helperText to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.helperText, null);

    const String helperText = 'Hint';
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            helperText: helperText,
            dropdownMenuEntries: menuEntries,
          ),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.helperText, helperText);
  });

  testWidgets('Passes selectedTrailingIcon to underlying DropdownMenu', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.selectedTrailingIcon, null);

    const Icon selectedTrailingIcon = Icon(Icons.abc);
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            selectedTrailingIcon: selectedTrailingIcon,
            dropdownMenuEntries: menuEntries,
          ),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.selectedTrailingIcon, selectedTrailingIcon);
  });

  testWidgets('Passes enableFilter to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.enableFilter, false);

    const bool enableFilter = true;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            enableFilter: enableFilter,
            dropdownMenuEntries: menuEntries,
          ),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.enableFilter, enableFilter);
  });

  testWidgets('Passes enableSearch to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.enableSearch, true);

    const bool enableSearch = false;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            enableSearch: enableSearch,
            dropdownMenuEntries: menuEntries,
          ),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.enableSearch, enableSearch);
  });

  testWidgets('Passes textStyle to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.textStyle, null);

    const TextStyle textStyle = TextStyle(fontWeight: FontWeight.bold);
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            textStyle: textStyle,
            dropdownMenuEntries: menuEntries,
          ),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.textStyle, textStyle);
  });

  testWidgets('Passes textAlign to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.textAlign, TextAlign.start);

    const TextAlign textAlign = TextAlign.center;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            textAlign: textAlign,
            dropdownMenuEntries: menuEntries,
          ),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.textAlign, textAlign);
  });

  testWidgets('Passes inputDecorationTheme to underlying DropdownMenu', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.inputDecorationTheme, null);

    const InputDecorationTheme inputDecorationTheme = InputDecorationTheme();
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            inputDecorationTheme: inputDecorationTheme,
            dropdownMenuEntries: menuEntries,
          ),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.inputDecorationTheme, inputDecorationTheme);
  });

  testWidgets('Passes menuStyle to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.menuStyle, null);

    const MenuStyle menuStyle = MenuStyle();
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            menuStyle: menuStyle,
            dropdownMenuEntries: menuEntries,
          ),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.menuStyle, menuStyle);
  });

  testWidgets('Passes controller to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.controller, null);

    final TextEditingController controller = TextEditingController();
    addTearDown(controller.dispose);

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            controller: controller,
            dropdownMenuEntries: menuEntries,
          ),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.controller, controller);
  });

  testWidgets('Passes requestFocusOnTap to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.requestFocusOnTap, null);

    const bool requestFocusOnTap = true;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            requestFocusOnTap: requestFocusOnTap,
            dropdownMenuEntries: menuEntries,
          ),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.requestFocusOnTap, requestFocusOnTap);
  });

  testWidgets('Passes expandedInsets to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.expandedInsets, null);

    const EdgeInsetsGeometry expandedInsets = EdgeInsets.zero;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            expandedInsets: expandedInsets,
            dropdownMenuEntries: menuEntries,
          ),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.expandedInsets, expandedInsets);
  });

  testWidgets('Passes alignmentOffset to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.alignmentOffset, null);

    const Offset alignmentOffset = Offset.zero;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            alignmentOffset: alignmentOffset,
            dropdownMenuEntries: menuEntries,
          ),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.alignmentOffset, alignmentOffset);
  });

  testWidgets('Passes filterCallback to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            enableFilter: true,
            dropdownMenuEntries: menuEntries,
          ),
        ),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.filterCallback, null);

    List<DropdownMenuEntry<MenuItem>> filterCallback(
      List<DropdownMenuEntry<MenuItem>> entries,
      String filter,
    ) {
      return entries;
    }

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            enableFilter: true,
            filterCallback: filterCallback,
            dropdownMenuEntries: menuEntries,
          ),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.filterCallback, filterCallback);
  });

  testWidgets('Passes searchCallback to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.searchCallback, null);

    int searchCallback(List<DropdownMenuEntry<MenuItem>> entries, String filter) {
      return 0;
    }

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            searchCallback: searchCallback,
            dropdownMenuEntries: menuEntries,
          ),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.searchCallback, searchCallback);
  });

  testWidgets('Passes inputFormatters to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.inputFormatters, null);

    final List<TextInputFormatter> inputFormatters = <TextInputFormatter>[];
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            inputFormatters: inputFormatters,
            dropdownMenuEntries: menuEntries,
          ),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.inputFormatters, inputFormatters);
  });

  testWidgets('Passes closeBehavior to underlying DropdownMenu', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: DropdownMenuFormField<MenuItem>(dropdownMenuEntries: menuEntries)),
      ),
    );

    // Check default value.
    DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.closeBehavior, DropdownMenuCloseBehavior.all);

    const DropdownMenuCloseBehavior closeBehavior = DropdownMenuCloseBehavior.self;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            closeBehavior: closeBehavior,
            dropdownMenuEntries: menuEntries,
          ),
        ),
      ),
    );

    dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.closeBehavior, closeBehavior);
  });

  testWidgets('Field state is correcly updated', (WidgetTester tester) async {
    final GlobalKey<FormFieldState<MenuItem>> fieldKey = GlobalKey<FormFieldState<MenuItem>>();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            key: fieldKey,
            dropdownMenuEntries: menuEntries,
            initialSelection: MenuItem.menuItem0,
          ),
        ),
      ),
    );

    await tester.tap(find.byType(DropdownMenu<MenuItem>));
    await tester.pump();

    await tester.tap(findMenuItem(MenuItem.menuItem1));
    await tester.pump();

    expect(fieldKey.currentState!.value, MenuItem.menuItem1);
  });

  testWidgets('onSaved callback is called when the field is outside a Form', (
    WidgetTester tester,
  ) async {
    final GlobalKey<FormFieldState<MenuItem>> fieldKey = GlobalKey<FormFieldState<MenuItem>>();

    MenuItem? savedValue = MenuItem.menuItem0;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            key: fieldKey,
            dropdownMenuEntries: menuEntries,
            initialSelection: savedValue,
            onSaved: (MenuItem? newValue) => savedValue = newValue,
          ),
        ),
      ),
    );

    await tester.tap(find.byType(DropdownMenu<MenuItem>));
    await tester.pump();

    await tester.tap(findMenuItem(MenuItem.menuItem1));
    await tester.pump();

    fieldKey.currentState!.save();
    await tester.pump();

    expect(savedValue, MenuItem.menuItem1);
  });

  testWidgets('onSaved callback is called when the field is inside a Form', (
    WidgetTester tester,
  ) async {
    final GlobalKey<FormState> formKey = GlobalKey<FormState>();

    MenuItem? savedValue = MenuItem.menuItem0;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Form(
            key: formKey,
            child: DropdownMenuFormField<MenuItem>(
              dropdownMenuEntries: menuEntries,
              initialSelection: savedValue,
              onSaved: (MenuItem? newValue) => savedValue = newValue,
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(DropdownMenu<MenuItem>));
    await tester.pump();

    await tester.tap(findMenuItem(MenuItem.menuItem1));
    await tester.pump();

    formKey.currentState!.save();
    await tester.pump();

    expect(savedValue, MenuItem.menuItem1);
  });

  testWidgets('Field can be reset', (WidgetTester tester) async {
    final GlobalKey<FormFieldState<MenuItem>> fieldKey = GlobalKey<FormFieldState<MenuItem>>();

    MenuItem? savedValue = MenuItem.menuItem0;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            key: fieldKey,
            dropdownMenuEntries: menuEntries,
            initialSelection: savedValue,
            onSaved: (MenuItem? newValue) => savedValue = newValue,
          ),
        ),
      ),
    );

    await tester.tap(find.byType(DropdownMenu<MenuItem>));
    await tester.pump();

    await tester.tap(findMenuItem(MenuItem.menuItem1));
    await tester.pump();

    expect(fieldKey.currentState!.value, MenuItem.menuItem1);

    fieldKey.currentState!.reset();
    await tester.pump();

    expect(fieldKey.currentState!.value, MenuItem.menuItem0);
  });

  testWidgets('isValid and hasError results are correct', (WidgetTester tester) async {
    final GlobalKey<FormFieldState<MenuItem>> fieldKey = GlobalKey<FormFieldState<MenuItem>>();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            key: fieldKey,
            dropdownMenuEntries: menuEntries,
            autovalidateMode: AutovalidateMode.always,
          ),
        ),
      ),
    );

    // No validation error.
    expect(fieldKey.currentState!.isValid, true);
    expect(fieldKey.currentState!.hasError, false);

    const String validationError = 'Required';
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            key: fieldKey,
            dropdownMenuEntries: menuEntries,
            autovalidateMode: AutovalidateMode.always,
            validator: (MenuItem? item) => validationError,
          ),
        ),
      ),
    );

    // Validation error.
    expect(fieldKey.currentState!.isValid, false);
    expect(fieldKey.currentState!.hasError, true);
  });

  testWidgets('Validation result is shown as error text', (WidgetTester tester) async {
    final GlobalKey<FormFieldState<MenuItem>> fieldKey = GlobalKey<FormFieldState<MenuItem>>();

    const String validationError = 'Required';
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: DropdownMenuFormField<MenuItem>(
            key: fieldKey,
            dropdownMenuEntries: menuEntries,
            autovalidateMode: AutovalidateMode.always,
            validator: (MenuItem? item) => validationError,
          ),
        ),
      ),
    );

    fieldKey.currentState!.validate();
    await tester.pump();

    expect(find.text('Required'), findsOneWidget);

    final DropdownMenu<MenuItem> dropdownMenu = tester.widget(find.byType(DropdownMenu<MenuItem>));
    expect(dropdownMenu.errorText, validationError);
  });
}
