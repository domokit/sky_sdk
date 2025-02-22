// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

/// Flutter code sample for a [RawMenuAnchor] that demonstrates
/// how to create a simple menu.
void main() {
  runApp(const RawMenuAnchorApp());
}

enum Animal {
  cat('Cat'),
  kitten('Kitten'),
  felisCatus('Felis catus'),
  dog('Dog');

  const Animal(this.label);
  final String label;
}

class RawMenuAnchorExample extends StatefulWidget {
  const RawMenuAnchorExample({super.key});

  @override
  State<RawMenuAnchorExample> createState() => _RawMenuAnchorExampleState();
}

class _RawMenuAnchorExampleState extends State<RawMenuAnchorExample> {
  final FocusNode focusNode = FocusNode();
  final MenuController controller = MenuController();
  Animal? _selectedAnimal;

  @override
  void dispose() {
    focusNode.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = Theme.of(context);
    return UnconstrainedBox(
      clipBehavior: Clip.hardEdge,
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          Text('Favorite Animal:', style: theme.textTheme.titleMedium),
          const SizedBox(width: 8),
          CustomMenu(
            controller: controller,
            focusNode: focusNode,
            anchor: FilledButton(
              focusNode: focusNode,
              style: FilledButton.styleFrom(fixedSize: const Size(172, 36)),
              onPressed: () {
                if (controller.isOpen) {
                  controller.close();
                } else {
                  controller.open();
                }
              },
              child: Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: <Widget>[
                  Expanded(flex: 3, child: Text(_selectedAnimal?.label ?? 'Select One')),
                  const Flexible(child: Icon(Icons.arrow_drop_down, size: 16)),
                ],
              ),
            ),
            children: <Widget>[
              for (final Animal animal in Animal.values)
                MenuItemButton(
                  autofocus: _selectedAnimal == animal,
                  onPressed: () {
                    setState(() {
                      _selectedAnimal = animal;
                    });
                    controller.close();
                  },
                  trailingIcon:
                      _selectedAnimal == animal ? const Icon(Icons.check, size: 20) : null,
                  child: Text(animal.label),
                ),
            ],
          ),
        ],
      ),
    );
  }
}

class CustomMenu extends StatelessWidget {
  const CustomMenu({
    super.key,
    required this.children,
    required this.anchor,
    required this.controller,
    required this.focusNode,
  });

  final List<Widget> children;
  final Widget anchor;
  final MenuController controller;
  final FocusNode focusNode;

  static const Map<ShortcutActivator, Intent> _shortcuts = <ShortcutActivator, Intent>{
    SingleActivator(LogicalKeyboardKey.gameButtonA): ActivateIntent(),
    SingleActivator(LogicalKeyboardKey.escape): DismissIntent(),
    SingleActivator(LogicalKeyboardKey.arrowDown): DirectionalFocusIntent(TraversalDirection.down),
    SingleActivator(LogicalKeyboardKey.arrowUp): DirectionalFocusIntent(TraversalDirection.up),
  };

  @override
  Widget build(BuildContext context) {
    return RawMenuAnchor(
      controller: controller,
      childFocusNode: focusNode,
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
                    constraints: const BoxConstraints(minWidth: 168),
                    padding: const EdgeInsets.symmetric(vertical: 6),
                    decoration: BoxDecoration(
                      color: Theme.of(context).colorScheme.surface,
                      borderRadius: BorderRadius.circular(6),
                      boxShadow: kElevationToShadow[4],
                    ),
                    child: Shortcuts(shortcuts: _shortcuts, child: Column(children: children)),
                  ),
                ),
              ),
            ),
          ),
        );
      },
      child: anchor,
    );
  }
}

class RawMenuAnchorApp extends StatelessWidget {
  const RawMenuAnchorApp({super.key});

  static const ButtonStyle menuButtonStyle = ButtonStyle(
    overlayColor: WidgetStatePropertyAll<Color>(Color.fromARGB(55, 139, 195, 255)),
    iconSize: WidgetStatePropertyAll<double>(17),
    padding: WidgetStatePropertyAll<EdgeInsets>(EdgeInsets.symmetric(horizontal: 12)),
  );

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData.from(
        colorScheme: ColorScheme.fromSeed(
          seedColor: Colors.blue,
          dynamicSchemeVariant: DynamicSchemeVariant.vibrant,
        ),
      ).copyWith(menuButtonTheme: const MenuButtonThemeData(style: menuButtonStyle)),
      home: const Scaffold(body: Center(child: RawMenuAnchorExample())),
    );
  }
}
