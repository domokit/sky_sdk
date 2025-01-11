// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

/// Flutter code sample for a [RawMenuAnchor] that shows a context menu.
/// The menu can be opened by right-clicking.
void main() => runApp(const ContextMenuApp());

class ContextMenuExample extends StatefulWidget {
  const ContextMenuExample({super.key});

  @override
  State<ContextMenuExample> createState() => _ContextMenuExampleState();
}

class _ContextMenuExampleState extends State<ContextMenuExample> {
  final MenuController controller = MenuController();
  bool _menuWasEnabled = false;
  String _selected = '';

  @override
  void initState() {
    super.initState();
    _disablePlatformContextMenu();
  }

  @override
  void dispose() {
    _enablePlatformContextMenu();
    super.dispose();
  }

  void _enablePlatformContextMenu() {
    if (!kIsWeb) {
      // Does nothing on non-web platforms.
      return;
    }
    if (_menuWasEnabled && !BrowserContextMenu.enabled) {
      BrowserContextMenu.enableContextMenu();
    }
  }

  Future<void> _disablePlatformContextMenu() async {
    if (!kIsWeb) {
      // Does nothing on non-web platforms.
      return;
    }
    _menuWasEnabled = BrowserContextMenu.enabled;
    if (_menuWasEnabled) {
      await BrowserContextMenu.disableContextMenu();
    }
  }

  void _handlePressed(String value) {
    setState(() {
      _selected = value;
    });
    controller.close();
  }

  @override
  Widget build(BuildContext context) {
    return RawMenuAnchor(
      controller: controller,
      alignmentOffset: const Offset(0, 6),
      panel: RawMenuPanel(
        constraints: const BoxConstraints(minWidth: 180),
        padding: const EdgeInsets.symmetric(vertical: 5),
        decoration: RawMenuPanel.lightSurfaceDecoration,
        menuChildren: <Widget>[
          MenuItemButton(
            autofocus: true,
            onPressed: () {
              _handlePressed('Undo');
            },
            leadingIcon: const Icon(Icons.undo),
            child: const Text('Undo'),
          ),
          MenuItemButton(
            onPressed: () {
              _handlePressed('Redo');
            },
            leadingIcon: const Icon(Icons.redo),
            child: const Text('Redo'),
          ),
          const Divider(thickness: 0.0, indent: 12, endIndent: 12),
          MenuItemButton(
            onPressed: () {
              _handlePressed('Cut');
            },
            leadingIcon: const Icon(Icons.cut),
            child: const Text('Cut'),
          ),
          MenuItemButton(
            onPressed: () {
              _handlePressed('Copy');
            },
            leadingIcon: const Icon(Icons.content_copy),
            child: const Text('Copy'),
          ),
          MenuItemButton(
            onPressed: () {
              _handlePressed('Paste');
            },
            leadingIcon: const Icon(Icons.content_paste),
            child: const Text('Paste'),
          ),
          const Divider(thickness: 0.0, indent: 12, endIndent: 12),
          RawMenuAnchor(
            padding: const EdgeInsetsDirectional.symmetric(vertical: 5),
            panel: RawMenuPanel(
              padding: const EdgeInsetsDirectional.symmetric(vertical: 5),
              constraints: const BoxConstraints(minWidth: 180),
              decoration: RawMenuPanel.lightSurfaceDecoration,
              menuChildren: <Widget>[
                MenuItemButton(
                  onPressed: () {
                    _handlePressed('Bold');
                  },
                  leadingIcon: const Icon(Icons.format_bold),
                  child: const Text('Bold'),
                ),
                MenuItemButton(
                  onPressed: () {
                    _handlePressed('Italic');
                  },
                  leadingIcon: const Icon(Icons.format_italic),
                  child: const Text('Italic'),
                ),
                MenuItemButton(
                  onPressed: () {
                    _handlePressed('Underline');
                  },
                  leadingIcon: const Icon(Icons.format_underline),
                  child: const Text('Underline'),
                ),
              ],
            ),
            builder: (BuildContext context, MenuController controller, Widget? child) {
              return MergeSemantics(
                child: Semantics(
                  expanded: controller.isOpen,
                  child: ColoredBox(
                    color: controller.isOpen ? const Color(0x0D1A1A1A) : Colors.transparent,
                    child: MenuItemButton(
                      onPressed: () {
                        if (controller.isOpen) {
                          controller.close();
                        } else {
                          controller.open();
                        }
                      },
                      leadingIcon: const Icon(Icons.text_format),
                      trailingIcon: const Icon(Icons.keyboard_arrow_right),
                      child: const Text('Format'),
                    ),
                  ),
                ),
              );
            },
          ),
        ],
      ),
      child: NestedWidget(
        child: Text(
          _selected.isEmpty ? 'Right-click me!' : 'Selected: $_selected',
          style: Theme.of(context).textTheme.displaySmall!.copyWith(color: Colors.white),
        ),
      ),
    );
  }
}

class NestedWidget extends StatelessWidget {
  const NestedWidget({super.key, this.child});
  final Widget? child;

  @override
  Widget build(BuildContext context) {
    // Long-press on mobile, secondary-click (right click) on desktop.
    GestureLongPressStartCallback? onLongPressStart;
    GestureTapDownCallback? onSecondaryTapDown;
    switch (defaultTargetPlatform) {
      case TargetPlatform.iOS:
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
        onLongPressStart = (LongPressStartDetails details) {
          MenuController.maybeOf(context)?.open(position: details.localPosition);
          HapticFeedback.heavyImpact();
        };
      case TargetPlatform.macOS:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        onSecondaryTapDown = (TapDownDetails details) {
          MenuController.maybeOf(context)?.open(position: details.localPosition);
        };
    }
    return GestureDetector(
      onSecondaryTapDown: onSecondaryTapDown,
      onLongPressStart: onLongPressStart,
      onTapDown: (TapDownDetails details) {
        MenuController.maybeOf(context)?.close();
      },
      child: ColoredBox(color: const Color(0xFF009DFF), child: Center(child: child)),
    );
  }
}

class ContextMenuApp extends StatelessWidget {
  const ContextMenuApp({super.key});

  static const ButtonStyle menuButtonStyle = ButtonStyle(
    splashFactory: InkSparkle.splashFactory,
    iconSize: WidgetStatePropertyAll<double>(17),
    overlayColor: WidgetStatePropertyAll<Color>(Color(0x12262627)),
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
      home: const Scaffold(body: ContextMenuExample()),
    );
  }
}
