// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
/// @docImport 'package:flutter/services.dart';
///
/// @docImport 'text_selection.dart';
library;

import 'framework.dart';

/// The buttons that can appear in a context menu by default.
///
/// See also:
///
///  * [ContextMenuButtonItem], which uses this enum to describe a button in a
///    context menu.
enum ContextMenuButtonType {
  /// A button that cuts the current text selection.
  cut,

  /// A button that copies the current text selection.
  copy,

  /// A button that pastes the clipboard contents into the focused text field.
  paste,

  /// A button that selects all the contents of the focused text field.
  selectAll,

  /// A button that deletes the current text selection.
  delete,

  /// A button that looks up the current text selection.
  lookUp,

  /// A button that launches a web search for the current text selection.
  searchWeb,

  /// A button that displays the share screen for the current text selection.
  share,

  /// A button for starting Live Text input.
  ///
  /// See also:
  ///  * [LiveText], where the availability of Live Text input can be obtained.
  ///  * [LiveTextInputStatusNotifier], where the status of Live Text can be listened to.
  liveTextInput,

  /// Anything other than the default button types.
  custom,
}

/// The type and callback for a context menu button.
///
/// See also:
///
///  * [AdaptiveTextSelectionToolbar], which can take a list of
///    ContextMenuButtonItems and create a platform-specific context menu with
///    the indicated buttons.
///  * [IOSSystemContextMenuItem], which serves a similar role but for
///    system-drawn context menu items on iOS.
@immutable
class ContextMenuButtonItem {
  /// Creates a const instance of [ContextMenuButtonItem].
  const ContextMenuButtonItem({
    required this.onPressed,
    this.type = ContextMenuButtonType.custom,
    this.label,
  });

  /// The callback to be called when the button is pressed.
  final VoidCallback? onPressed;

  /// The type of button this represents.
  final ContextMenuButtonType type;

  /// The label to display on the button.
  ///
  /// If a [type] other than [ContextMenuButtonType.custom] is given
  /// and a label is not provided, then the default label for that type for the
  /// platform will be looked up.
  final String? label;

  /// Creates a new [ContextMenuButtonItem] with the provided parameters
  /// overridden.
  ContextMenuButtonItem copyWith({
    VoidCallback? onPressed,
    ContextMenuButtonType? type,
    String? label,
  }) {
    return ContextMenuButtonItem(
      onPressed: onPressed ?? this.onPressed,
      type: type ?? this.type,
      label: label ?? this.label,
    );
  }

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is ContextMenuButtonItem &&
        other.label == label &&
        other.onPressed == onPressed &&
        other.type == type;
  }

  @override
  int get hashCode => Object.hash(label, onPressed, type);

  @override
  String toString() => 'ContextMenuButtonItem $type, $label';
}
