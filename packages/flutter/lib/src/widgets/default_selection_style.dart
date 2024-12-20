// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'editable_text.dart';
/// @docImport 'text.dart';
library;

import 'basic.dart';
import 'framework.dart';
import 'inherited_theme.dart';

// Examples can assume:
// late BuildContext context;

/// The selection style to apply to descendant [EditableText] widgets which
/// don't have an explicit style.
///
/// {@macro flutter.cupertino.CupertinoApp.defaultSelectionStyle}
///
/// {@macro flutter.material.MaterialApp.defaultSelectionStyle}
///
/// See also:
///  * [TextSelectionTheme]: which also creates a [DefaultSelectionStyle] for
///    the subtree.
class DefaultSelectionStyle extends InheritedTheme {
  /// Creates a default selection style widget that specifies the selection
  /// properties for all widgets below it in the widget tree.
  const DefaultSelectionStyle({
    super.key,
    this.cursorColor,
    this.selectionColor,
    this.mouseCursor,
    required super.child,
  });

  /// A const-constructable default selection style that provides fallback
  /// values (null).
  ///
  /// Returned from [of] when the given [BuildContext] doesn't have an enclosing
  /// default selection style.
  ///
  /// This constructor creates a [DefaultTextStyle] with an invalid [child],
  /// which means the constructed value cannot be incorporated into the tree.
  const DefaultSelectionStyle.fallback({super.key})
    : cursorColor = null,
      selectionColor = null,
      mouseCursor = null,
      super(child: const _NullWidget());

  /// Creates a default selection style that overrides the selection styles in
  /// scope at this point in the widget tree.
  ///
  /// Any Arguments that are not null replace the corresponding properties on the
  /// default selection style for the [BuildContext] where the widget is inserted.
  static Widget merge({
    Key? key,
    Color? cursorColor,
    Color? selectionColor,
    MouseCursor? mouseCursor,
    required Widget child,
  }) {
    return Builder(
      builder: (BuildContext context) {
        final DefaultSelectionStyle parent = DefaultSelectionStyle.of(context);
        return DefaultSelectionStyle(
          key: key,
          cursorColor: cursorColor ?? parent.cursorColor,
          selectionColor: selectionColor ?? parent.selectionColor,
          mouseCursor: mouseCursor ?? parent.mouseCursor,
          child: child,
        );
      },
    );
  }

  /// The default cursor and selection color (semi-transparent grey).
  ///
  /// This is the color that the [Text] widget uses when the specified selection
  /// color is null.
  static const Color defaultColor = Color(0x80808080);

  /// The color of the text field's cursor.
  ///
  /// The cursor indicates the current location of the text insertion point in
  /// the field.
  final Color? cursorColor;

  /// The background color of selected text.
  final Color? selectionColor;

  /// The [MouseCursor] for mouse pointers hovering over selectable Text widgets.
  ///
  /// If this property is null, [SystemMouseCursors.text] will be used.
  final MouseCursor? mouseCursor;

  /// The closest instance of this class that encloses the given context.
  ///
  /// If no such instance exists, returns an instance created by
  /// [DefaultSelectionStyle.fallback], which contains fallback values.
  ///
  /// Typical usage is as follows:
  ///
  /// ```dart
  /// DefaultSelectionStyle style = DefaultSelectionStyle.of(context);
  /// ```
  static DefaultSelectionStyle of(BuildContext context) {
    return context.dependOnInheritedWidgetOfExactType<DefaultSelectionStyle>() ??
        const DefaultSelectionStyle.fallback();
  }

  @override
  Widget wrap(BuildContext context, Widget child) {
    return DefaultSelectionStyle(
      cursorColor: cursorColor,
      selectionColor: selectionColor,
      mouseCursor: mouseCursor,
      child: child,
    );
  }

  @override
  bool updateShouldNotify(DefaultSelectionStyle oldWidget) {
    return cursorColor != oldWidget.cursorColor ||
        selectionColor != oldWidget.selectionColor ||
        mouseCursor != oldWidget.mouseCursor;
  }
}

class _NullWidget extends StatelessWidget {
  const _NullWidget();

  @override
  Widget build(BuildContext context) {
    throw FlutterError(
      'A DefaultSelectionStyle constructed with DefaultSelectionStyle.fallback cannot be incorporated into the widget tree, '
      'it is meant only to provide a fallback value returned by DefaultSelectionStyle.of() '
      'when no enclosing default selection style is present in a BuildContext.',
    );
  }
}
