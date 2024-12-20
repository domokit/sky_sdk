// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'button_style.dart';
/// @docImport 'text_selection_toolbar.dart';
library;

import 'package:flutter/widgets.dart';

import 'color_scheme.dart';
import 'constants.dart';
import 'text_button.dart';
import 'theme.dart';

enum _TextSelectionToolbarItemPosition {
  /// The first item among multiple in the menu.
  first,

  /// One of several items, not the first or last.
  middle,

  /// The last item among multiple in the menu.
  last,

  /// The only item in the menu.
  only,
}

/// A button styled like a Material native Android text selection menu button.
class TextSelectionToolbarTextButton extends StatelessWidget {
  /// Creates an instance of TextSelectionToolbarTextButton.
  const TextSelectionToolbarTextButton({
    super.key,
    required this.child,
    required this.padding,
    this.onPressed,
    this.alignment,
  });

  // These values were eyeballed to match the native text selection menu on a
  // Pixel 2 running Android 10.
  static const double _kMiddlePadding = 9.5;
  static const double _kEndPadding = 14.5;

  /// {@template flutter.material.TextSelectionToolbarTextButton.child}
  /// The child of this button.
  ///
  /// Usually a [Text].
  /// {@endtemplate}
  final Widget child;

  /// {@template flutter.material.TextSelectionToolbarTextButton.onPressed}
  /// Called when this button is pressed.
  /// {@endtemplate}
  final VoidCallback? onPressed;

  /// The padding between the button's edge and its child.
  ///
  /// In a standard Material [TextSelectionToolbar], the padding depends on the
  /// button's position within the toolbar.
  ///
  /// See also:
  ///
  ///  * [getPadding], which calculates the standard padding based on the
  ///    button's position.
  ///  * [ButtonStyle.padding], which is where this padding is applied.
  final EdgeInsets padding;

  /// The alignment of the button's child.
  ///
  /// By default, this will be [Alignment.center].
  ///
  /// See also:
  ///
  ///  * [ButtonStyle.alignment], which is where this alignment is applied.
  final AlignmentGeometry? alignment;

  /// Returns the standard padding for a button at index out of a total number
  /// of buttons.
  ///
  /// Standard Material [TextSelectionToolbar]s have buttons with different
  /// padding depending on their position in the toolbar.
  static EdgeInsets getPadding(int index, int total) {
    assert(total > 0 && index >= 0 && index < total);
    final _TextSelectionToolbarItemPosition position = _getPosition(index, total);
    return EdgeInsets.only(left: _getLeftPadding(position), right: _getRightPadding(position));
  }

  static double _getLeftPadding(_TextSelectionToolbarItemPosition position) {
    if (position == _TextSelectionToolbarItemPosition.first ||
        position == _TextSelectionToolbarItemPosition.only) {
      return _kEndPadding;
    }
    return _kMiddlePadding;
  }

  static double _getRightPadding(_TextSelectionToolbarItemPosition position) {
    if (position == _TextSelectionToolbarItemPosition.last ||
        position == _TextSelectionToolbarItemPosition.only) {
      return _kEndPadding;
    }
    return _kMiddlePadding;
  }

  static _TextSelectionToolbarItemPosition _getPosition(int index, int total) {
    if (index == 0) {
      return total == 1
          ? _TextSelectionToolbarItemPosition.only
          : _TextSelectionToolbarItemPosition.first;
    }
    if (index == total - 1) {
      return _TextSelectionToolbarItemPosition.last;
    }
    return _TextSelectionToolbarItemPosition.middle;
  }

  /// Returns a copy of the current [TextSelectionToolbarTextButton] instance
  /// with specific overrides.
  TextSelectionToolbarTextButton copyWith({
    Widget? child,
    VoidCallback? onPressed,
    EdgeInsets? padding,
    AlignmentGeometry? alignment,
  }) {
    return TextSelectionToolbarTextButton(
      onPressed: onPressed ?? this.onPressed,
      padding: padding ?? this.padding,
      alignment: alignment ?? this.alignment,
      child: child ?? this.child,
    );
  }

  // These colors were taken from a screenshot of a Pixel 6 emulator running
  // Android API level 34.
  static const Color _defaultForegroundColorLight = Color(0xff000000);
  static const Color _defaultForegroundColorDark = Color(0xffffffff);

  // The background color is hardcoded to transparent by default so the buttons
  // are the color of the container behind them. For example TextSelectionToolbar
  // hardcodes the color value, and TextSelectionToolbarTextButtons that are its
  // children become that color.
  static const Color _defaultBackgroundColorTransparent = Color(0x00000000);

  static Color _getForegroundColor(ColorScheme colorScheme) {
    final bool isDefaultOnSurface = switch (colorScheme.brightness) {
      Brightness.light => identical(ThemeData().colorScheme.onSurface, colorScheme.onSurface),
      Brightness.dark => identical(ThemeData.dark().colorScheme.onSurface, colorScheme.onSurface),
    };
    if (!isDefaultOnSurface) {
      return colorScheme.onSurface;
    }
    return switch (colorScheme.brightness) {
      Brightness.light => _defaultForegroundColorLight,
      Brightness.dark => _defaultForegroundColorDark,
    };
  }

  @override
  Widget build(BuildContext context) {
    final ColorScheme colorScheme = Theme.of(context).colorScheme;
    return TextButton(
      style: TextButton.styleFrom(
        backgroundColor: _defaultBackgroundColorTransparent,
        foregroundColor: _getForegroundColor(colorScheme),
        shape: const RoundedRectangleBorder(),
        minimumSize: const Size(kMinInteractiveDimension, kMinInteractiveDimension),
        padding: padding,
        alignment: alignment,
        textStyle: const TextStyle(
          // This value was eyeballed from a screenshot of a Pixel 6 emulator
          // running Android API level 34.
          fontWeight: FontWeight.w400,
        ),
      ),
      onPressed: onPressed,
      child: child,
    );
  }
}
