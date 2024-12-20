// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

/// [HighlightFocus] is a helper widget for giving a child focus
/// allowing tab-navigation.
/// Wrap your widget as [child] of a [HighlightFocus] widget.
class HighlightFocus extends StatefulWidget {
  const HighlightFocus({
    super.key,
    required this.onPressed,
    required this.child,
    this.highlightColor,
    this.borderColor,
    this.hasFocus = true,
    this.debugLabel,
  });

  /// [onPressed] is called when you press space, enter, or numpad-enter
  /// when the widget is focused.
  final VoidCallback onPressed;

  /// [child] is your widget.
  final Widget child;

  /// [highlightColor] is the color filled in the border when the widget
  /// is focused.
  /// Use [Colors.transparent] if you do not want one.
  /// Use an opacity less than 1 to make the underlying widget visible.
  final Color? highlightColor;

  /// [borderColor] is the color of the border when the widget is focused.
  final Color? borderColor;

  /// [hasFocus] is true when focusing on the widget is allowed.
  /// Set to false if you want the child to skip focus.
  final bool hasFocus;

  final String? debugLabel;

  @override
  State<HighlightFocus> createState() => _HighlightFocusState();
}

class _HighlightFocusState extends State<HighlightFocus> {
  late bool isFocused;

  @override
  void initState() {
    isFocused = false;
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    final Color highlightColor =
        widget.highlightColor ?? Theme.of(context).colorScheme.primary.withOpacity(0.5);
    final Color borderColor = widget.borderColor ?? Theme.of(context).colorScheme.onPrimary;

    final BoxDecoration highlightedDecoration = BoxDecoration(
      color: highlightColor,
      border: Border.all(color: borderColor, width: 2, strokeAlign: BorderSide.strokeAlignOutside),
    );

    return Focus(
      canRequestFocus: widget.hasFocus,
      debugLabel: widget.debugLabel,
      onFocusChange: (bool newValue) {
        setState(() {
          isFocused = newValue;
        });
      },
      onKeyEvent: (FocusNode node, KeyEvent event) {
        if ((event is KeyDownEvent || event is KeyRepeatEvent) &&
            (event.logicalKey == LogicalKeyboardKey.space ||
                event.logicalKey == LogicalKeyboardKey.enter ||
                event.logicalKey == LogicalKeyboardKey.numpadEnter)) {
          widget.onPressed();
          return KeyEventResult.handled;
        } else {
          return KeyEventResult.ignored;
        }
      },
      child: Container(
        foregroundDecoration: isFocused ? highlightedDecoration : null,
        child: widget.child,
      ),
    );
  }
}
