// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [MenuAnchor].

void main() => runApp(const MenuAnchorApp());

// This is the type used by the menu below.
enum SampleItem { itemOne, itemTwo, itemThree }

class MenuAnchorApp extends StatelessWidget {
  const MenuAnchorApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: MenuAnchorExample());
  }
}

class MenuAnchorExample extends StatefulWidget {
  const MenuAnchorExample({super.key});

  @override
  State<MenuAnchorExample> createState() => _MenuAnchorExampleState();
}

class _MenuAnchorExampleState extends State<MenuAnchorExample> {
  SampleItem? selectedMenu;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('MenuAnchorButton'),
        backgroundColor: Theme.of(context).primaryColorLight,
      ),
      body: Center(
        child: MenuAnchor(
          builder: (BuildContext context, MenuController controller, Widget? child) {
            return IconButton(
              onPressed: () {
                if (controller.isOpen) {
                  controller.close();
                } else {
                  controller.open();
                }
              },
              icon: const Icon(Icons.more_horiz),
              tooltip: 'Show menu',
            );
          },
          menuChildren: List<MenuItemButton>.generate(
            3,
            (int index) => MenuItemButton(
              onPressed: () => setState(() => selectedMenu = SampleItem.values[index]),
              child: Text('Item ${index + 1}'),
            ),
          ),
        ),
      ),
    );
  }
}
