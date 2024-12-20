// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

void main() {
  runApp(const MaterialApp(title: 'Menu Tester', home: Material(child: Home())));
}

class Home extends StatefulWidget {
  const Home({super.key});

  @override
  State<Home> createState() => _HomeState();
}

class _HomeState extends State<Home> {
  final MenuController _controller = MenuController();
  VisualDensity _density = VisualDensity.standard;
  TextDirection _textDirection = TextDirection.ltr;
  double _extraPadding = 0;
  bool _addItem = false;
  bool _accelerators = true;
  bool _transparent = false;
  bool _funkyTheme = false;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    MenuThemeData menuTheme = MenuTheme.of(context);
    MenuBarThemeData menuBarTheme = MenuBarTheme.of(context);
    MenuButtonThemeData menuButtonTheme = MenuButtonTheme.of(context);
    if (_funkyTheme) {
      menuTheme = const MenuThemeData(
        style: MenuStyle(
          shape: MaterialStatePropertyAll<OutlinedBorder>(
            RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(10))),
          ),
          backgroundColor: MaterialStatePropertyAll<Color?>(Colors.blue),
          elevation: MaterialStatePropertyAll<double?>(10),
          padding: MaterialStatePropertyAll<EdgeInsetsDirectional>(EdgeInsetsDirectional.all(20)),
        ),
      );
      menuButtonTheme = const MenuButtonThemeData(
        style: ButtonStyle(
          shape: MaterialStatePropertyAll<OutlinedBorder>(StadiumBorder()),
          backgroundColor: MaterialStatePropertyAll<Color?>(Colors.green),
          foregroundColor: MaterialStatePropertyAll<Color?>(Colors.white),
        ),
      );
      menuBarTheme = const MenuBarThemeData(
        style: MenuStyle(
          shape: MaterialStatePropertyAll<OutlinedBorder>(RoundedRectangleBorder()),
          backgroundColor: MaterialStatePropertyAll<Color?>(Colors.blue),
          elevation: MaterialStatePropertyAll<double?>(10),
          padding: MaterialStatePropertyAll<EdgeInsetsDirectional>(EdgeInsetsDirectional.all(20)),
        ),
      );
    }
    return SafeArea(
      child: Padding(
        padding: EdgeInsets.all(_extraPadding),
        child: Directionality(
          textDirection: _textDirection,
          child: Theme(
            data: theme.copyWith(
              visualDensity: _density,
              menuTheme:
                  _transparent
                      ? MenuThemeData(
                        style: MenuStyle(
                          backgroundColor: MaterialStatePropertyAll<Color>(
                            Colors.blue.withOpacity(0.12),
                          ),
                          elevation: const MaterialStatePropertyAll<double>(0),
                        ),
                      )
                      : menuTheme,
              menuBarTheme: menuBarTheme,
              menuButtonTheme: menuButtonTheme,
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                _TestMenus(
                  menuController: _controller,
                  accelerators: _accelerators,
                  addItem: _addItem,
                ),
                Expanded(
                  child: SingleChildScrollView(
                    child: _Controls(
                      menuController: _controller,
                      density: _density,
                      addItem: _addItem,
                      accelerators: _accelerators,
                      transparent: _transparent,
                      funkyTheme: _funkyTheme,
                      extraPadding: _extraPadding,
                      textDirection: _textDirection,
                      onDensityChanged: (VisualDensity value) {
                        setState(() {
                          _density = value;
                        });
                      },
                      onTextDirectionChanged: (TextDirection value) {
                        setState(() {
                          _textDirection = value;
                        });
                      },
                      onExtraPaddingChanged: (double value) {
                        setState(() {
                          _extraPadding = value;
                        });
                      },
                      onAddItemChanged: (bool value) {
                        setState(() {
                          _addItem = value;
                        });
                      },
                      onAcceleratorsChanged: (bool value) {
                        setState(() {
                          _accelerators = value;
                        });
                      },
                      onTransparentChanged: (bool value) {
                        setState(() {
                          _transparent = value;
                        });
                      },
                      onFunkyThemeChanged: (bool value) {
                        setState(() {
                          _funkyTheme = value;
                        });
                      },
                    ),
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

class _Controls extends StatefulWidget {
  const _Controls({
    required this.density,
    required this.textDirection,
    required this.extraPadding,
    this.addItem = false,
    this.accelerators = true,
    this.transparent = false,
    this.funkyTheme = false,
    required this.onDensityChanged,
    required this.onTextDirectionChanged,
    required this.onExtraPaddingChanged,
    required this.onAddItemChanged,
    required this.onAcceleratorsChanged,
    required this.onTransparentChanged,
    required this.onFunkyThemeChanged,
    required this.menuController,
  });

  final VisualDensity density;
  final TextDirection textDirection;
  final double extraPadding;
  final bool addItem;
  final bool accelerators;
  final bool transparent;
  final bool funkyTheme;
  final ValueChanged<VisualDensity> onDensityChanged;
  final ValueChanged<TextDirection> onTextDirectionChanged;
  final ValueChanged<double> onExtraPaddingChanged;
  final ValueChanged<bool> onAddItemChanged;
  final ValueChanged<bool> onAcceleratorsChanged;
  final ValueChanged<bool> onTransparentChanged;
  final ValueChanged<bool> onFunkyThemeChanged;
  final MenuController menuController;

  @override
  State<_Controls> createState() => _ControlsState();
}

class _ControlsState extends State<_Controls> {
  final FocusNode _focusNode = FocusNode(debugLabel: 'Floating');

  @override
  void dispose() {
    _focusNode.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: SingleChildScrollView(
        child: Column(
          children: <Widget>[
            MenuAnchor(
              childFocusNode: _focusNode,
              style: const MenuStyle(alignment: AlignmentDirectional.topEnd),
              alignmentOffset: const Offset(100, -8),
              menuChildren: <Widget>[
                MenuItemButton(
                  shortcut: TestMenu.standaloneMenu1.shortcut,
                  onPressed: () {
                    _itemSelected(TestMenu.standaloneMenu1);
                  },
                  child: MenuAcceleratorLabel(TestMenu.standaloneMenu1.label),
                ),
                MenuItemButton(
                  leadingIcon: const Icon(Icons.send),
                  trailingIcon: const Icon(Icons.mail),
                  onPressed: () {
                    _itemSelected(TestMenu.standaloneMenu2);
                  },
                  child: MenuAcceleratorLabel(TestMenu.standaloneMenu2.label),
                ),
              ],
              builder: (BuildContext context, MenuController controller, Widget? child) {
                return TextButton(
                  focusNode: _focusNode,
                  onPressed: () {
                    if (controller.isOpen) {
                      controller.close();
                    } else {
                      controller.open();
                    }
                  },
                  child: child!,
                );
              },
              child: const MenuAcceleratorLabel('Open Menu'),
            ),
            ConstrainedBox(
              constraints: const BoxConstraints(maxWidth: 400),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: <Widget>[
                  _ControlSlider(
                    label: 'Extra Padding: ${widget.extraPadding.toStringAsFixed(1)}',
                    value: widget.extraPadding,
                    max: 40,
                    divisions: 20,
                    onChanged: (double value) {
                      widget.onExtraPaddingChanged(value);
                    },
                  ),
                  _ControlSlider(
                    label: 'Horizontal Density: ${widget.density.horizontal.toStringAsFixed(1)}',
                    value: widget.density.horizontal,
                    max: 4,
                    min: -4,
                    divisions: 12,
                    onChanged: (double value) {
                      widget.onDensityChanged(
                        VisualDensity(horizontal: value, vertical: widget.density.vertical),
                      );
                    },
                  ),
                  _ControlSlider(
                    label: 'Vertical Density: ${widget.density.vertical.toStringAsFixed(1)}',
                    value: widget.density.vertical,
                    max: 4,
                    min: -4,
                    divisions: 12,
                    onChanged: (double value) {
                      widget.onDensityChanged(
                        VisualDensity(horizontal: widget.density.horizontal, vertical: value),
                      );
                    },
                  ),
                ],
              ),
            ),
            Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: <Widget>[
                Row(
                  mainAxisSize: MainAxisSize.min,
                  children: <Widget>[
                    Checkbox(
                      value: widget.textDirection == TextDirection.rtl,
                      onChanged: (bool? value) {
                        if (value ?? false) {
                          widget.onTextDirectionChanged(TextDirection.rtl);
                        } else {
                          widget.onTextDirectionChanged(TextDirection.ltr);
                        }
                      },
                    ),
                    const Text('RTL Text'),
                  ],
                ),
                Row(
                  mainAxisSize: MainAxisSize.min,
                  children: <Widget>[
                    Checkbox(
                      value: widget.addItem,
                      onChanged: (bool? value) {
                        if (value ?? false) {
                          widget.onAddItemChanged(true);
                        } else {
                          widget.onAddItemChanged(false);
                        }
                      },
                    ),
                    const Text('Add Item'),
                  ],
                ),
                Row(
                  mainAxisSize: MainAxisSize.min,
                  children: <Widget>[
                    Checkbox(
                      value: widget.accelerators,
                      onChanged: (bool? value) {
                        if (value ?? false) {
                          widget.onAcceleratorsChanged(true);
                        } else {
                          widget.onAcceleratorsChanged(false);
                        }
                      },
                    ),
                    const Text('Enable Accelerators'),
                  ],
                ),
                Row(
                  mainAxisSize: MainAxisSize.min,
                  children: <Widget>[
                    Checkbox(
                      value: widget.transparent,
                      onChanged: (bool? value) {
                        if (value ?? false) {
                          widget.onTransparentChanged(true);
                        } else {
                          widget.onTransparentChanged(false);
                        }
                      },
                    ),
                    const Text('Transparent'),
                  ],
                ),
                Row(
                  mainAxisSize: MainAxisSize.min,
                  children: <Widget>[
                    Checkbox(
                      value: widget.funkyTheme,
                      onChanged: (bool? value) {
                        if (value ?? false) {
                          widget.onFunkyThemeChanged(true);
                        } else {
                          widget.onFunkyThemeChanged(false);
                        }
                      },
                    ),
                    const Text('Funky Theme'),
                  ],
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  void _itemSelected(TestMenu item) {
    debugPrint('App: Selected item ${item.label}');
  }
}

class _ControlSlider extends StatelessWidget {
  const _ControlSlider({
    required this.label,
    required this.value,
    required this.onChanged,
    this.min = 0,
    this.max = 1,
    this.divisions,
  });

  final String label;
  final double value;
  final ValueChanged<double> onChanged;
  final double min;
  final double max;
  final int? divisions;

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisAlignment: MainAxisAlignment.end,
      children: <Widget>[
        Container(
          alignment: AlignmentDirectional.centerEnd,
          constraints: const BoxConstraints(minWidth: 150),
          child: Text(label),
        ),
        Expanded(
          child: Slider(
            value: value,
            min: min,
            max: max,
            divisions: divisions,
            onChanged: onChanged,
          ),
        ),
      ],
    );
  }
}

class _TestMenus extends StatefulWidget {
  const _TestMenus({required this.menuController, this.addItem = false, this.accelerators = false});

  final MenuController menuController;
  final bool addItem;
  final bool accelerators;

  @override
  State<_TestMenus> createState() => _TestMenusState();
}

class _TestMenusState extends State<_TestMenus> {
  final TextEditingController textController = TextEditingController();
  bool? checkboxState = false;
  TestMenu? radioValue;
  ShortcutRegistryEntry? _shortcutsEntry;

  void _itemSelected(TestMenu item) {
    debugPrint('App: Selected item ${item.label}');
  }

  void _openItem(TestMenu item) {
    debugPrint('App: Opened item ${item.label}');
  }

  void _closeItem(TestMenu item) {
    debugPrint('App: Closed item ${item.label}');
  }

  void _setRadio(TestMenu? item) {
    debugPrint('App: Set Radio item ${item?.label}');
    setState(() {
      radioValue = item;
    });
  }

  void _setCheck(TestMenu item) {
    debugPrint('App: Set Checkbox item ${item.label}');
    setState(() {
      checkboxState = switch (checkboxState) {
        false => true,
        true => null,
        null => false,
      };
    });
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _shortcutsEntry?.dispose();
    final Map<ShortcutActivator, Intent> shortcuts = <ShortcutActivator, Intent>{};
    for (final TestMenu item in TestMenu.values) {
      if (item.shortcut == null) {
        continue;
      }
      switch (item) {
        case TestMenu.radioMenu1:
        case TestMenu.radioMenu2:
        case TestMenu.radioMenu3:
          shortcuts[item.shortcut!] = VoidCallbackIntent(() => _setRadio(item));
        case TestMenu.subMenu1:
          shortcuts[item.shortcut!] = VoidCallbackIntent(() => _setCheck(item));
        case TestMenu.mainMenu1:
        case TestMenu.mainMenu2:
        case TestMenu.mainMenu3:
        case TestMenu.mainMenu4:
        case TestMenu.subMenu2:
        case TestMenu.subMenu3:
        case TestMenu.subMenu4:
        case TestMenu.subMenu5:
        case TestMenu.subMenu6:
        case TestMenu.subMenu7:
        case TestMenu.subMenu8:
        case TestMenu.subSubMenu1:
        case TestMenu.subSubMenu2:
        case TestMenu.subSubMenu3:
        case TestMenu.subSubSubMenu1:
        case TestMenu.testButton:
        case TestMenu.standaloneMenu1:
        case TestMenu.standaloneMenu2:
          shortcuts[item.shortcut!] = VoidCallbackIntent(() => _itemSelected(item));
      }
    }
    _shortcutsEntry = ShortcutRegistry.of(context).addAll(shortcuts);
  }

  @override
  void dispose() {
    _shortcutsEntry?.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Row(
      children: <Widget>[
        Expanded(
          child: MenuBar(
            controller: widget.menuController,
            children: createTestMenus(
              onPressed: _itemSelected,
              onOpen: _openItem,
              onClose: _closeItem,
              onCheckboxChanged: (TestMenu menu, bool? value) {
                _setCheck(menu);
              },
              onRadioChanged: _setRadio,
              checkboxValue: checkboxState,
              radioValue: radioValue,
              menuController: widget.menuController,
              textEditingController: textController,
              includeExtraGroups: widget.addItem,
              accelerators: widget.accelerators,
            ),
          ),
        ),
      ],
    );
  }
}

List<Widget> createTestMenus({
  void Function(TestMenu)? onPressed,
  void Function(TestMenu, bool?)? onCheckboxChanged,
  void Function(TestMenu?)? onRadioChanged,
  void Function(TestMenu)? onOpen,
  void Function(TestMenu)? onClose,
  Map<TestMenu, MenuSerializableShortcut> shortcuts = const <TestMenu, MenuSerializableShortcut>{},
  bool? checkboxValue,
  TestMenu? radioValue,
  MenuController? menuController,
  TextEditingController? textEditingController,
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

  Widget checkboxMenuButton(
    TestMenu menu, {
    bool enabled = true,
    bool tristate = false,
    Widget? leadingIcon,
    Widget? trailingIcon,
    Key? key,
  }) {
    return CheckboxMenuButton(
      key: key,
      value: checkboxValue,
      tristate: tristate,
      onChanged:
          enabled && onCheckboxChanged != null
              ? (bool? value) => onCheckboxChanged(menu, value)
              : null,
      shortcut: menu.shortcut,
      trailingIcon: trailingIcon,
      child: accelerators ? MenuAcceleratorLabel(menu.acceleratorLabel) : Text(menu.label),
    );
  }

  Widget radioMenuButton(
    TestMenu menu, {
    bool enabled = true,
    bool toggleable = false,
    Widget? leadingIcon,
    Widget? trailingIcon,
    Key? key,
  }) {
    return RadioMenuButton<TestMenu>(
      key: key,
      groupValue: radioValue,
      value: menu,
      toggleable: toggleable,
      onChanged: enabled && onRadioChanged != null ? onRadioChanged : null,
      shortcut: menu.shortcut,
      trailingIcon: trailingIcon,
      child: accelerators ? MenuAcceleratorLabel(menu.acceleratorLabel) : Text(menu.label),
    );
  }

  final List<Widget> result = <Widget>[
    submenuButton(
      TestMenu.mainMenu1,
      menuChildren: <Widget>[
        checkboxMenuButton(
          TestMenu.subMenu1,
          tristate: true,
          trailingIcon: const Icon(Icons.assessment),
        ),
        radioMenuButton(
          TestMenu.radioMenu1,
          toggleable: true,
          trailingIcon: const Icon(Icons.assessment),
        ),
        radioMenuButton(
          TestMenu.radioMenu2,
          toggleable: true,
          trailingIcon: const Icon(Icons.assessment),
        ),
        radioMenuButton(
          TestMenu.radioMenu3,
          toggleable: true,
          trailingIcon: const Icon(Icons.assessment),
        ),
        menuItemButton(
          TestMenu.subMenu2,
          leadingIcon: const Icon(Icons.send),
          trailingIcon: const Icon(Icons.mail),
        ),
      ],
    ),
    submenuButton(
      TestMenu.mainMenu2,
      menuChildren: <Widget>[
        MenuAcceleratorCallbackBinding(
          onInvoke:
              onPressed != null
                  ? () {
                    onPressed.call(TestMenu.testButton);
                    menuController?.close();
                  }
                  : null,
          child: TextButton(
            onPressed:
                onPressed != null
                    ? () {
                      onPressed.call(TestMenu.testButton);
                      menuController?.close();
                    }
                    : null,
            child:
                accelerators
                    ? MenuAcceleratorLabel(TestMenu.testButton.acceleratorLabel)
                    : Text(TestMenu.testButton.label),
          ),
        ),
        menuItemButton(TestMenu.subMenu3),
      ],
    ),
    submenuButton(
      TestMenu.mainMenu3,
      menuChildren: <Widget>[
        menuItemButton(TestMenu.subMenu8),
        MenuItemButton(
          onPressed: () {
            debugPrint('Focused Item: $primaryFocus');
          },
          child: const Text('Print Focused Item'),
        ),
      ],
    ),
    submenuButton(
      TestMenu.mainMenu4,
      menuChildren: <Widget>[
        MenuItemButton(
          onPressed: () {
            debugPrint('Activated text input item with ${textEditingController?.text} as a value.');
          },
          child: SizedBox(
            width: 200,
            child: TextField(
              controller: textEditingController,
              onSubmitted: (String value) {
                debugPrint('String $value submitted.');
              },
            ),
          ),
        ),
        submenuButton(
          TestMenu.subMenu5,
          menuChildren: <Widget>[
            menuItemButton(TestMenu.subSubMenu1),
            menuItemButton(TestMenu.subSubMenu2),
            if (includeExtraGroups)
              submenuButton(
                TestMenu.subSubMenu3,
                menuChildren: <Widget>[
                  for (int i = 0; i < 100; ++i)
                    MenuItemButton(onPressed: () {}, child: Text('Menu Item $i')),
                ],
              ),
          ],
        ),
        menuItemButton(TestMenu.subMenu6, enabled: false),
        menuItemButton(TestMenu.subMenu7),
        menuItemButton(TestMenu.subMenu7),
        menuItemButton(TestMenu.subMenu8),
      ],
    ),
  ];
  return result;
}

enum TestMenu {
  mainMenu1('Menu 1'),
  mainMenu2('M&enu &2'),
  mainMenu3('Me&nu &3'),
  mainMenu4('Men&u &4'),
  radioMenu1('Radio Menu One', SingleActivator(LogicalKeyboardKey.digit1, control: true)),
  radioMenu2('Radio Menu Two', SingleActivator(LogicalKeyboardKey.digit2, control: true)),
  radioMenu3('Radio Menu Three', SingleActivator(LogicalKeyboardKey.digit3, control: true)),
  subMenu1('Sub Menu &1', SingleActivator(LogicalKeyboardKey.keyB, control: true)),
  subMenu2('Sub Menu &2'),
  subMenu3('Sub Menu &3', SingleActivator(LogicalKeyboardKey.enter, control: true)),
  subMenu4('Sub Menu &4'),
  subMenu5('Sub Menu &5'),
  subMenu6('Sub Menu &6', SingleActivator(LogicalKeyboardKey.tab, control: true)),
  subMenu7('Sub Menu &7'),
  subMenu8('Sub Menu &8'),
  subSubMenu1('Sub Sub Menu &1', SingleActivator(LogicalKeyboardKey.f10, control: true)),
  subSubMenu2('Sub Sub Menu &2'),
  subSubMenu3('Sub Sub Menu &3'),
  subSubSubMenu1('Sub Sub Sub Menu &1', SingleActivator(LogicalKeyboardKey.f11, control: true)),
  testButton('&TEST && &&& Button &'),
  standaloneMenu1('Standalone Menu &1', SingleActivator(LogicalKeyboardKey.keyC, control: true)),
  standaloneMenu2('Standalone Menu &2');

  const TestMenu(this.acceleratorLabel, [this.shortcut]);
  final MenuSerializableShortcut? shortcut;
  final String acceleratorLabel;
  // Strip the accelerator markers.
  String get label => MenuAcceleratorLabel.stripAcceleratorMarkers(acceleratorLabel);
}
