// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

/// Flutter code sample for a [RawMenuAnchorGroup] that demonstrates
/// how to create a menu bar for a document editor.
void main() => runApp(const RawMenuAnchorGroupApp());

class MenuItem {
  const MenuItem(this.label, {this.leading, this.children});
  final String label;
  final Widget? leading;
  final List<MenuItem>? children;
}

const List<MenuItem> menuItems = <MenuItem>[
  MenuItem(
    'File',
    children: <MenuItem>[
      MenuItem('New', leading: Icon(Icons.edit_document)),
      MenuItem('Open', leading: Icon(Icons.folder)),
      MenuItem('Print', leading: Icon(Icons.print)),
      MenuItem('Share', leading: Icon(Icons.share)),
    ],
  ),
  MenuItem(
    'Edit',
    children: <MenuItem>[
      MenuItem('Undo', leading: Icon(Icons.undo)),
      MenuItem('Redo', leading: Icon(Icons.redo)),
      MenuItem('Cut', leading: Icon(Icons.cut)),
      MenuItem('Copy', leading: Icon(Icons.copy)),
      MenuItem('Paste', leading: Icon(Icons.paste)),
    ],
  ),
  MenuItem(
    'View',
    children: <MenuItem>[
      MenuItem('Zoom In', leading: Icon(Icons.zoom_in)),
      MenuItem('Zoom Out', leading: Icon(Icons.zoom_out)),
      MenuItem('Fit', leading: Icon(Icons.fullscreen)),
    ],
  ),
  MenuItem(
    'Tools',
    children: <MenuItem>[
      MenuItem('Spelling', leading: Icon(Icons.spellcheck)),
      MenuItem('Grammar', leading: Icon(Icons.text_format)),
      MenuItem('Thesaurus', leading: Icon(Icons.book_outlined)),
      MenuItem('Dictionary', leading: Icon(Icons.book)),
    ],
  ),
];

class RawMenuAnchorGroupExample extends StatefulWidget {
  const RawMenuAnchorGroupExample({super.key});

  @override
  State<RawMenuAnchorGroupExample> createState() => _RawMenuAnchorGroupExampleState();
}

class _RawMenuAnchorGroupExampleState extends State<RawMenuAnchorGroupExample> {
  final MenuController controller = MenuController();
  MenuItem? _selected;
  List<FocusNode> focusNodes = List<FocusNode>.generate(
    menuItems.length,
    (int index) => FocusNode(),
  );

  @override
  void dispose() {
    for (final FocusNode focusNode in focusNodes) {
      focusNode.dispose();
    }
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    final TextStyle titleStyle = theme.textTheme.titleMedium!;
    return Center(
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          if (_selected != null) Text('Selected: ${_selected!.label}', style: titleStyle),
          UnconstrainedBox(
            clipBehavior: Clip.hardEdge,
            child: RawMenuAnchorGroup(
              controller: controller,
              child: Row(
                children: <Widget>[
                  for (int i = 0; i < menuItems.length; i++)
                    CustomSubmenu(
                      focusNode: focusNodes[i],
                      anchor: Builder(
                        builder: (BuildContext context) {
                          final MenuController submenuController = MenuController.maybeOf(context)!;
                          final MenuItem item = menuItems[i];
                          final ButtonStyle openBackground = MenuItemButton.styleFrom(
                            backgroundColor: const Color(0x0D1A1A1A),
                          );
                          return MergeSemantics(
                            child: Semantics(
                              expanded: controller.isOpen,
                              child: MenuItemButton(
                                style: submenuController.isOpen ? openBackground : null,
                                onHover: (bool value) {
                                  // If any submenu in the menu bar is already open, other
                                  // submenus should open on hover. Otherwise, blur the menu item
                                  // button if the menu button is no longer hovered.
                                  if (controller.isOpen) {
                                    if (value) {
                                      submenuController.open();
                                    }
                                  } else if (!value) {
                                    Focus.of(context).unfocus();
                                  }
                                },
                                onPressed: () {
                                  if (submenuController.isOpen) {
                                    submenuController.close();
                                  } else {
                                    submenuController.open();
                                  }
                                },
                                leadingIcon: item.leading,
                                child: Text(item.label),
                              ),
                            ),
                          );
                        },
                      ),
                      children: <Widget>[
                        for (final MenuItem child in menuItems[i].children ?? <MenuItem>[])
                          MenuItemButton(
                            onPressed: () {
                              setState(() {
                                _selected = child;
                              });

                              // Close the menu bar after a selection.
                              controller.close();
                            },
                            leadingIcon: child.leading,
                            child: Text(child.label),
                          ),
                      ],
                    ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class CustomSubmenu extends StatefulWidget {
  const CustomSubmenu({
    super.key,
    required this.children,
    required this.anchor,
    required this.focusNode,
  });

  final List<Widget> children;
  final Widget anchor;
  final FocusNode focusNode;

  static const Map<ShortcutActivator, Intent> _shortcuts = <ShortcutActivator, Intent>{
    SingleActivator(LogicalKeyboardKey.gameButtonA): ActivateIntent(),
    SingleActivator(LogicalKeyboardKey.escape): DismissIntent(),
    SingleActivator(LogicalKeyboardKey.arrowDown): DirectionalFocusIntent(TraversalDirection.down),
    SingleActivator(LogicalKeyboardKey.arrowUp): DirectionalFocusIntent(TraversalDirection.up),
  };

  @override
  State<CustomSubmenu> createState() => _CustomSubmenuState();
}

class _CustomSubmenuState extends State<CustomSubmenu> {
  final MenuController menuController = MenuController();

  @override
  Widget build(BuildContext context) {
    return RawMenuAnchor(
      controller: menuController,
      childFocusNode: widget.focusNode,
      overlayBuilder: (BuildContext context, RawMenuOverlayInfo info) {
        return Positioned(
          top: info.anchorRect.bottom + 4,
          left: info.anchorRect.left,
          // The overlay will be treated as a dialog.
          child: Semantics(
            scopesRoute: true,
            explicitChildNodes: true,
            child: TapRegion(
              groupId: info.tapRegionGroupId,
              onTapOutside: (PointerDownEvent event) {
                MenuController.maybeOf(context)?.close();
              },
              child: FocusScope(
                child: IntrinsicWidth(
                  child: Container(
                    clipBehavior: Clip.antiAlias,
                    constraints: const BoxConstraints(minWidth: 160),
                    padding: const EdgeInsets.symmetric(vertical: 6),
                    decoration: BoxDecoration(
                      color: Theme.of(context).colorScheme.surface,
                      borderRadius: BorderRadius.circular(6),
                      boxShadow: kElevationToShadow[4],
                    ),
                    child: Shortcuts(
                      shortcuts: CustomSubmenu._shortcuts,
                      child: Column(
                        mainAxisSize: MainAxisSize.min,
                        crossAxisAlignment: CrossAxisAlignment.stretch,
                        children: widget.children,
                      ),
                    ),
                  ),
                ),
              ),
            ),
          ),
        );
      },
      child: widget.anchor,
    );
  }
}

class RawMenuAnchorGroupApp extends StatelessWidget {
  const RawMenuAnchorGroupApp({super.key});

  static const ButtonStyle menuButtonStyle = ButtonStyle(
    splashFactory: InkSparkle.splashFactory,
    iconSize: WidgetStatePropertyAll<double>(17),
    overlayColor: WidgetStatePropertyAll<Color>(Color(0x0D1A1A1A)),
    padding: WidgetStatePropertyAll<EdgeInsets>(EdgeInsets.symmetric(horizontal: 12)),
    textStyle: WidgetStatePropertyAll<TextStyle>(TextStyle(fontSize: 14)),
    visualDensity: VisualDensity(
      horizontal: VisualDensity.minimumDensity,
      vertical: VisualDensity.minimumDensity,
    ),
  );

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData.from(
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.blue),
      ).copyWith(menuButtonTheme: const MenuButtonThemeData(style: menuButtonStyle)),
      home: const Scaffold(body: RawMenuAnchorGroupExample()),
    );
  }
}
