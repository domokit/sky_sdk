// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This sample demonstrates allowing a context menu to be shown in a widget
// subtree in response to user gestures.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

void main() => runApp(const ContextMenuControllerExampleApp());

/// A builder that includes an Offset to draw the context menu at.
typedef ContextMenuBuilder = Widget Function(BuildContext context, Offset offset);

class ContextMenuControllerExampleApp extends StatefulWidget {
  const ContextMenuControllerExampleApp({super.key});

  @override
  State<ContextMenuControllerExampleApp> createState() => _ContextMenuControllerExampleAppState();
}

class _ContextMenuControllerExampleAppState extends State<ContextMenuControllerExampleApp> {
  void _showDialog(BuildContext context) {
    Navigator.of(context).push(
      DialogRoute<void>(
        context: context,
        builder: (BuildContext context) => const AlertDialog(title: Text('You clicked print!')),
      ),
    );
  }

  @override
  void initState() {
    super.initState();
    // On web, disable the browser's context menu since this example uses a custom
    // Flutter-rendered context menu.
    if (kIsWeb) {
      BrowserContextMenu.disableContextMenu();
    }
  }

  @override
  void dispose() {
    if (kIsWeb) {
      BrowserContextMenu.enableContextMenu();
    }
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('Context menu outside of text')),
        body: _ContextMenuRegion(
          contextMenuBuilder: (BuildContext context, Offset offset) {
            // The custom context menu will look like the default context menu
            // on the current platform with a single 'Print' button.
            return AdaptiveTextSelectionToolbar.buttonItems(
              anchors: TextSelectionToolbarAnchors(primaryAnchor: offset),
              buttonItems: <ContextMenuButtonItem>[
                ContextMenuButtonItem(
                  onPressed: () {
                    ContextMenuController.removeAny();
                    _showDialog(context);
                  },
                  label: 'Print',
                ),
              ],
            );
          },
          // In this case this wraps a big open space in a GestureDetector in
          // order to show the context menu, but it could also wrap a single
          // widget like an Image to give it a context menu.
          child: ListView(
            children: <Widget>[
              Container(height: 20.0),
              const Text(
                'Right click (desktop) or long press (mobile) anywhere, not just on this text, to show the custom menu.',
              ),
            ],
          ),
        ),
      ),
    );
  }
}

/// Shows and hides the context menu based on user gestures.
///
/// By default, shows the menu on right clicks and long presses.
class _ContextMenuRegion extends StatefulWidget {
  /// Creates an instance of [_ContextMenuRegion].
  const _ContextMenuRegion({required this.child, required this.contextMenuBuilder});

  /// Builds the context menu.
  final ContextMenuBuilder contextMenuBuilder;

  /// The child widget that will be listened to for gestures.
  final Widget child;

  @override
  State<_ContextMenuRegion> createState() => _ContextMenuRegionState();
}

class _ContextMenuRegionState extends State<_ContextMenuRegion> {
  Offset? _longPressOffset;

  final ContextMenuController _contextMenuController = ContextMenuController();

  static bool get _longPressEnabled {
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
      case TargetPlatform.iOS:
        return true;
      case TargetPlatform.macOS:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        return false;
    }
  }

  void _onSecondaryTapUp(TapUpDetails details) {
    _show(details.globalPosition);
  }

  void _onTap() {
    if (!_contextMenuController.isShown) {
      return;
    }
    _hide();
  }

  void _onLongPressStart(LongPressStartDetails details) {
    _longPressOffset = details.globalPosition;
  }

  void _onLongPress() {
    assert(_longPressOffset != null);
    _show(_longPressOffset!);
    _longPressOffset = null;
  }

  void _show(Offset position) {
    _contextMenuController.show(
      context: context,
      contextMenuBuilder: (BuildContext context) {
        return widget.contextMenuBuilder(context, position);
      },
    );
  }

  void _hide() {
    _contextMenuController.remove();
  }

  @override
  void dispose() {
    _hide();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      behavior: HitTestBehavior.opaque,
      onSecondaryTapUp: _onSecondaryTapUp,
      onTap: _onTap,
      onLongPress: _longPressEnabled ? _onLongPress : null,
      onLongPressStart: _longPressEnabled ? _onLongPressStart : null,
      child: widget.child,
    );
  }
}
