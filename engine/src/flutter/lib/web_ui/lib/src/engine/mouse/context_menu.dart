// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../dom.dart';
import 'prevent_default.dart';

/// Controls the browser's context menu in the given [element].
class ContextMenu {
  ContextMenu(this.element);

  final DomElement element;

  /// False when the context menu has been disabled, otherwise true.
  bool _enabled = true;

  /// Disables the browser's context menu for this [element].
  ///
  /// By default, when a Flutter web app starts, the context menu is enabled.
  ///
  /// Can be re-enabled by calling [enable].
  void disable() {
    if (!_enabled) {
      return;
    }
    _enabled = false;
    element.addEventListener('contextmenu', preventDefaultListener);
  }

  /// Enables the browser's context menu for this [element].
  ///
  /// By default, when a Flutter web app starts, the context menu is already
  /// enabled. Typically, this method would be used after calling
  /// [disable] to first disable it.
  void enable() {
    if (_enabled) {
      return;
    }
    _enabled = true;
    element.removeEventListener('contextmenu', preventDefaultListener);
  }
}
