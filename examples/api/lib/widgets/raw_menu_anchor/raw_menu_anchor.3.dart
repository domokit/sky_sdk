// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for a [RawMenuAnchor.node] that demonstrates
/// how to create a menu bar for a document editor.
void main() => runApp(const MenuNodeApp());

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
      MenuItem(
        'Share',
        leading: Icon(Icons.share),
        children: <MenuItem>[
          MenuItem('Email', leading: Icon(Icons.email)),
          MenuItem('Message', leading: Icon(Icons.message)),
          MenuItem('Copy Link', leading: Icon(Icons.link)),
        ],
      ),
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

class MenuNodeExample extends StatefulWidget {
  const MenuNodeExample({super.key});

  @override
  State<MenuNodeExample> createState() => _MenuNodeExampleState();
}

class _MenuNodeExampleState extends State<MenuNodeExample> {
  final MenuController rootController = MenuController();
  static const WidgetStatePropertyAll<Color> menuButtonHoverColor = WidgetStatePropertyAll<Color>(
    Color(0x0D1A1A1A),
  );
  static const EdgeInsets padding = EdgeInsets.symmetric(vertical: 5);
  MenuItem? _selected;

  ButtonStyle? get menuButtonStyle => Theme.of(context).menuButtonTheme.style;
  TextStyle? get titleStyle => Theme.of(context).textTheme.titleMedium;

  RawMenuAnchor _buildSubmenu(MenuItem option, {double depth = 0}) {
    return RawMenuAnchor(
      padding: padding,
      alignmentOffset: depth == 0 ? const Offset(0, 5) : const Offset(-4, 0),
      panel: RawMenuPanel(
        padding: padding,
        constraints: const BoxConstraints(minWidth: 180),
        decoration: RawMenuPanel.lightSurfaceDecoration,
        menuChildren: <Widget>[
          for (final MenuItem child in option.children!)
            if (child.children != null)
              _buildSubmenu(child, depth: depth + 1)
            else
              MenuItemButton(
                onPressed: () {
                  setState(() {
                    _selected = child;
                  });
                  rootController.close();
                },
                leadingIcon: child.leading,
                child: Text(child.label),
              ),
        ],
      ),
      builder: (BuildContext context, MenuController controller, Widget? child) {
        return MergeSemantics(
          child: Semantics(
            expanded: controller.isOpen,
            child: MenuItemButton(
              style:
                  controller.isOpen
                      ? menuButtonStyle?.copyWith(backgroundColor: menuButtonHoverColor)
                      : menuButtonStyle,
              onHover: (bool value) {
                if (rootController.isOpen) {
                  if (value) {
                    controller.open();
                  }
                } else if (!value) {
                  Focus.of(context).unfocus();
                }
              },
              onPressed: () {
                if (controller.isOpen) {
                  controller.close();
                } else {
                  controller.open();
                }
              },
              leadingIcon: option.leading,
              trailingIcon: depth > 0 ? const Icon(Icons.keyboard_arrow_right) : null,
              child: Text(option.label),
            ),
          ),
        );
      },
    );
  }

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          if (_selected != null) Text('Selected: ${_selected!.label}', style: titleStyle),
          RawMenuAnchor.node(
            controller: rootController,
            child: Row(
              mainAxisSize: MainAxisSize.min,
              children: <RawMenuAnchor>[for (final MenuItem item in menuItems) _buildSubmenu(item)],
            ),
            builder: (BuildContext context, Widget? child) {
              return SizedBox(height: 44, child: child);
            },
          ),
        ],
      ),
    );
  }
}

class MenuNodeApp extends StatelessWidget {
  const MenuNodeApp({super.key});

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
      home: const Scaffold(body: MenuNodeExample()),
    );
  }
}
