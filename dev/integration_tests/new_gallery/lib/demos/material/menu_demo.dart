// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery_localizations.dart';
import 'material_demo_types.dart';

enum SimpleValue { one, two, three }

enum CheckedValue { one, two, three, four }

class MenuDemo extends StatefulWidget {
  const MenuDemo({super.key, required this.type});

  final MenuDemoType type;

  @override
  State<MenuDemo> createState() => _MenuDemoState();
}

class _MenuDemoState extends State<MenuDemo> {
  void showInSnackBar(String value) {
    ScaffoldMessenger.of(context).hideCurrentSnackBar();
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(value)));
  }

  @override
  Widget build(BuildContext context) {
    Widget demo;
    switch (widget.type) {
      case MenuDemoType.contextMenu:
        demo = _ContextMenuDemo(showInSnackBar: showInSnackBar);
      case MenuDemoType.sectionedMenu:
        demo = _SectionedMenuDemo(showInSnackBar: showInSnackBar);
      case MenuDemoType.simpleMenu:
        demo = _SimpleMenuDemo(showInSnackBar: showInSnackBar);
      case MenuDemoType.checklistMenu:
        demo = _ChecklistMenuDemo(showInSnackBar: showInSnackBar);
    }

    return Scaffold(
      appBar: AppBar(
        title: Text(GalleryLocalizations.of(context)!.demoMenuTitle),
        automaticallyImplyLeading: false,
      ),
      body: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 20),
        child: Center(child: demo),
      ),
    );
  }
}

// BEGIN menuDemoContext

// Pressing the PopupMenuButton on the right of this item shows
// a simple menu with one disabled item. Typically the contents
// of this "contextual menu" would reflect the app's state.
class _ContextMenuDemo extends StatelessWidget {
  const _ContextMenuDemo({required this.showInSnackBar});

  final void Function(String value) showInSnackBar;

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return ListTile(
      title: Text(localizations.demoMenuAnItemWithAContextMenuButton),
      trailing: PopupMenuButton<String>(
        padding: EdgeInsets.zero,
        onSelected: (String value) => showInSnackBar(localizations.demoMenuSelected(value)),
        itemBuilder:
            (BuildContext context) => <PopupMenuItem<String>>[
              PopupMenuItem<String>(
                value: localizations.demoMenuContextMenuItemOne,
                child: Text(localizations.demoMenuContextMenuItemOne),
              ),
              PopupMenuItem<String>(
                enabled: false,
                child: Text(localizations.demoMenuADisabledMenuItem),
              ),
              PopupMenuItem<String>(
                value: localizations.demoMenuContextMenuItemThree,
                child: Text(localizations.demoMenuContextMenuItemThree),
              ),
            ],
      ),
    );
  }
}

// END

// BEGIN menuDemoSectioned

// Pressing the PopupMenuButton on the right of this item shows
// a menu whose items have text labels and icons and a divider
// That separates the first three items from the last one.
class _SectionedMenuDemo extends StatelessWidget {
  const _SectionedMenuDemo({required this.showInSnackBar});

  final void Function(String value) showInSnackBar;

  @override
  Widget build(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return ListTile(
      title: Text(localizations.demoMenuAnItemWithASectionedMenu),
      trailing: PopupMenuButton<String>(
        padding: EdgeInsets.zero,
        onSelected: (String value) => showInSnackBar(localizations.demoMenuSelected(value)),
        itemBuilder:
            (BuildContext context) => <PopupMenuEntry<String>>[
              PopupMenuItem<String>(
                value: localizations.demoMenuPreview,
                child: ListTile(
                  leading: const Icon(Icons.visibility),
                  title: Text(localizations.demoMenuPreview),
                ),
              ),
              PopupMenuItem<String>(
                value: localizations.demoMenuShare,
                child: ListTile(
                  leading: const Icon(Icons.person_add),
                  title: Text(localizations.demoMenuShare),
                ),
              ),
              PopupMenuItem<String>(
                value: localizations.demoMenuGetLink,
                child: ListTile(
                  leading: const Icon(Icons.link),
                  title: Text(localizations.demoMenuGetLink),
                ),
              ),
              const PopupMenuDivider(),
              PopupMenuItem<String>(
                value: localizations.demoMenuRemove,
                child: ListTile(
                  leading: const Icon(Icons.delete),
                  title: Text(localizations.demoMenuRemove),
                ),
              ),
            ],
      ),
    );
  }
}

// END

// BEGIN menuDemoSimple

// This entire list item is a PopupMenuButton. Tapping anywhere shows
// a menu whose current value is highlighted and aligned over the
// list item's center line.
class _SimpleMenuDemo extends StatefulWidget {
  const _SimpleMenuDemo({required this.showInSnackBar});

  final void Function(String value) showInSnackBar;

  @override
  _SimpleMenuDemoState createState() => _SimpleMenuDemoState();
}

class _SimpleMenuDemoState extends State<_SimpleMenuDemo> {
  late SimpleValue _simpleValue;

  void showAndSetMenuSelection(BuildContext context, SimpleValue value) {
    setState(() {
      _simpleValue = value;
    });
    widget.showInSnackBar(
      GalleryLocalizations.of(context)!.demoMenuSelected(simpleValueToString(context, value)),
    );
  }

  String simpleValueToString(BuildContext context, SimpleValue value) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return <SimpleValue, String>{
      SimpleValue.one: localizations.demoMenuItemValueOne,
      SimpleValue.two: localizations.demoMenuItemValueTwo,
      SimpleValue.three: localizations.demoMenuItemValueThree,
    }[value]!;
  }

  @override
  void initState() {
    super.initState();
    _simpleValue = SimpleValue.two;
  }

  @override
  Widget build(BuildContext context) {
    return PopupMenuButton<SimpleValue>(
      padding: EdgeInsets.zero,
      initialValue: _simpleValue,
      onSelected: (SimpleValue value) => showAndSetMenuSelection(context, value),
      itemBuilder:
          (BuildContext context) => <PopupMenuItem<SimpleValue>>[
            PopupMenuItem<SimpleValue>(
              value: SimpleValue.one,
              child: Text(simpleValueToString(context, SimpleValue.one)),
            ),
            PopupMenuItem<SimpleValue>(
              value: SimpleValue.two,
              child: Text(simpleValueToString(context, SimpleValue.two)),
            ),
            PopupMenuItem<SimpleValue>(
              value: SimpleValue.three,
              child: Text(simpleValueToString(context, SimpleValue.three)),
            ),
          ],
      child: ListTile(
        title: Text(GalleryLocalizations.of(context)!.demoMenuAnItemWithASimpleMenu),
        subtitle: Text(simpleValueToString(context, _simpleValue)),
      ),
    );
  }
}

// END

// BEGIN menuDemoChecklist

// Pressing the PopupMenuButton on the right of this item shows a menu
// whose items have checked icons that reflect this app's state.
class _ChecklistMenuDemo extends StatefulWidget {
  const _ChecklistMenuDemo({required this.showInSnackBar});

  final void Function(String value) showInSnackBar;

  @override
  _ChecklistMenuDemoState createState() => _ChecklistMenuDemoState();
}

class _RestorableCheckedValues extends RestorableProperty<Set<CheckedValue>> {
  Set<CheckedValue> _checked = <CheckedValue>{};

  void check(CheckedValue value) {
    _checked.add(value);
    notifyListeners();
  }

  void uncheck(CheckedValue value) {
    _checked.remove(value);
    notifyListeners();
  }

  bool isChecked(CheckedValue value) => _checked.contains(value);

  Iterable<String> checkedValuesToString(BuildContext context) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return _checked.map((CheckedValue value) {
      return <CheckedValue, String>{
        CheckedValue.one: localizations.demoMenuOne,
        CheckedValue.two: localizations.demoMenuTwo,
        CheckedValue.three: localizations.demoMenuThree,
        CheckedValue.four: localizations.demoMenuFour,
      }[value]!;
    });
  }

  @override
  Set<CheckedValue> createDefaultValue() => _checked;

  @override
  Set<CheckedValue> initWithValue(Set<CheckedValue> a) {
    _checked = a;
    return _checked;
  }

  @override
  Object toPrimitives() => _checked.map((CheckedValue value) => value.index).toList();

  @override
  Set<CheckedValue> fromPrimitives(Object? data) {
    final List<dynamic> checkedValues = data! as List<dynamic>;
    return Set<CheckedValue>.from(
      checkedValues.map<CheckedValue>((dynamic id) {
        return CheckedValue.values[id as int];
      }),
    );
  }
}

class _ChecklistMenuDemoState extends State<_ChecklistMenuDemo> with RestorationMixin {
  final _RestorableCheckedValues _checkedValues =
      _RestorableCheckedValues()..check(CheckedValue.three);

  @override
  String get restorationId => 'checklist_menu_demo';

  @override
  void restoreState(RestorationBucket? oldBucket, bool initialRestore) {
    registerForRestoration(_checkedValues, 'checked_values');
  }

  void showCheckedMenuSelections(BuildContext context, CheckedValue value) {
    if (_checkedValues.isChecked(value)) {
      setState(() {
        _checkedValues.uncheck(value);
      });
    } else {
      setState(() {
        _checkedValues.check(value);
      });
    }

    widget.showInSnackBar(
      GalleryLocalizations.of(
        context,
      )!.demoMenuChecked(_checkedValues.checkedValuesToString(context)),
    );
  }

  String checkedValueToString(BuildContext context, CheckedValue value) {
    final GalleryLocalizations localizations = GalleryLocalizations.of(context)!;
    return <CheckedValue, String>{
      CheckedValue.one: localizations.demoMenuOne,
      CheckedValue.two: localizations.demoMenuTwo,
      CheckedValue.three: localizations.demoMenuThree,
      CheckedValue.four: localizations.demoMenuFour,
    }[value]!;
  }

  @override
  void dispose() {
    _checkedValues.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return ListTile(
      title: Text(GalleryLocalizations.of(context)!.demoMenuAnItemWithAChecklistMenu),
      trailing: PopupMenuButton<CheckedValue>(
        padding: EdgeInsets.zero,
        onSelected: (CheckedValue value) => showCheckedMenuSelections(context, value),
        itemBuilder:
            (BuildContext context) => <PopupMenuItem<CheckedValue>>[
              CheckedPopupMenuItem<CheckedValue>(
                value: CheckedValue.one,
                checked: _checkedValues.isChecked(CheckedValue.one),
                child: Text(checkedValueToString(context, CheckedValue.one)),
              ),
              CheckedPopupMenuItem<CheckedValue>(
                value: CheckedValue.two,
                enabled: false,
                checked: _checkedValues.isChecked(CheckedValue.two),
                child: Text(checkedValueToString(context, CheckedValue.two)),
              ),
              CheckedPopupMenuItem<CheckedValue>(
                value: CheckedValue.three,
                checked: _checkedValues.isChecked(CheckedValue.three),
                child: Text(checkedValueToString(context, CheckedValue.three)),
              ),
              CheckedPopupMenuItem<CheckedValue>(
                value: CheckedValue.four,
                checked: _checkedValues.isChecked(CheckedValue.four),
                child: Text(checkedValueToString(context, CheckedValue.four)),
              ),
            ],
      ),
    );
  }
}

// END
