// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/services.dart';
library;

import 'framework.dart';
import 'inherited_theme.dart';
import 'navigator.dart';
import 'overlay.dart';

/// Builds and manages a context menu at a given location.
///
/// There can only ever be one context menu shown at a given time in the entire
/// app. Calling [show] on one instance of this class will hide any other
/// shown instances.
///
/// {@tool dartpad}
/// This example shows how to use a GestureDetector to show a context menu
/// anywhere in a widget subtree that receives a right click or long press.
///
/// ** See code in examples/api/lib/material/context_menu/context_menu_controller.0.dart **
/// {@end-tool}
///
/// See also:
///
///   * [BrowserContextMenu], which allows the browser's context menu on web to
///     be disabled and Flutter-rendered context menus to appear.
class ContextMenuController {
  /// Creates a context menu that can be shown with [show].
  ContextMenuController({this.onRemove});

  /// Called when this menu is removed.
  final VoidCallback? onRemove;

  /// The currently shown instance, if any.
  static ContextMenuController? _shownInstance;

  // The OverlayEntry is static because only one context menu can be displayed
  // at one time.
  static OverlayEntry? _menuOverlayEntry;

  /// Shows the given context menu.
  ///
  /// Since there can only be one shown context menu at a time, calling this
  /// will also remove any other context menu that is visible.
  void show({
    required BuildContext context,
    required WidgetBuilder contextMenuBuilder,
    Widget? debugRequiredFor,
  }) {
    removeAny();
    final OverlayState overlayState = Overlay.of(
      context,
      rootOverlay: true,
      debugRequiredFor: debugRequiredFor,
    );
    final CapturedThemes capturedThemes = InheritedTheme.capture(
      from: context,
      to: Navigator.maybeOf(context)?.context,
    );

    _menuOverlayEntry = OverlayEntry(
      builder: (BuildContext context) {
        return capturedThemes.wrap(contextMenuBuilder(context));
      },
    );
    overlayState.insert(_menuOverlayEntry!);
    _shownInstance = this;
  }

  /// Remove the currently shown context menu from the UI.
  ///
  /// Does nothing if no context menu is currently shown.
  ///
  /// If a menu is removed, and that menu provided an [onRemove] callback when
  /// it was created, then that callback will be called.
  ///
  /// See also:
  ///
  ///  * [remove], which removes only the current instance.
  static void removeAny() {
    _menuOverlayEntry?.remove();
    _menuOverlayEntry?.dispose();
    _menuOverlayEntry = null;
    if (_shownInstance != null) {
      _shownInstance!.onRemove?.call();
      _shownInstance = null;
    }
  }

  /// True if and only if this menu is currently being shown.
  bool get isShown => _shownInstance == this;

  /// Cause the underlying [OverlayEntry] to rebuild during the next pipeline
  /// flush.
  ///
  /// It's necessary to call this function if the output of `contextMenuBuilder`
  /// has changed.
  ///
  /// Errors if the context menu is not currently shown.
  ///
  /// See also:
  ///
  ///  * [OverlayEntry.markNeedsBuild]
  void markNeedsBuild() {
    assert(isShown);
    _menuOverlayEntry?.markNeedsBuild();
  }

  /// Remove this menu from the UI.
  ///
  /// Does nothing if this instance is not currently shown. In other words, if
  /// another context menu is currently shown, that menu will not be removed.
  ///
  /// This method should only be called once. The instance cannot be shown again
  /// after removing. Create a new instance.
  ///
  /// If an [onRemove] method was given to this instance, it will be called.
  ///
  /// See also:
  ///
  ///  * [removeAny], which removes any shown instance of the context menu.
  void remove() {
    if (!isShown) {
      return;
    }
    removeAny();
  }
}
