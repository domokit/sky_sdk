// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'implicit_animations.dart';
library;

import 'package:flutter/animation.dart';
import 'package:flutter/foundation.dart';

import 'framework.dart';
import 'inherited_notifier.dart';
import 'inherited_theme.dart';

/// Defines a default configuration for animation [Duration]s and [Curve]s.
///
/// If an [ImplicitlyAnimatedWidget]'s duration or curve is not defined,
/// it will default to the values specified in the closest ancestor
/// `DefaultAnimationStyle`, if one exists; otherwise it will use
/// [fallbackDuration] and/or [fallbackCurve] respectively.
sealed class DefaultAnimationStyle implements Widget {
  /// Creates a default configuration for animation [Duration]s and [Curve]s.
  ///
  /// The `mergeWithAncestor` parameter defaults to `true`; it determines
  /// whether null values from the provided `style` are filled in with
  /// non-null properties from further up in the widget tree, if applicable.
  const factory DefaultAnimationStyle({
    Key? key,
    required AnimationStyle style,
    bool mergeWithAncestor,
    required Widget child,
  }) = _DefaultAnimationStyle;

  /// Creates an [InheritedWidget] that defines a default style
  /// based on a `ValueListenable` object, typically a [ValueNotifier].
  ///
  /// Supplying a notifier directly allows sending notifications without
  /// needing to rebuild the widget.
  ///
  /// The notifier's style is not merged with ancestors in the widget tree.
  const factory DefaultAnimationStyle.notifier({
    Key? key,
    required ValueListenable<AnimationStyle> notifier,
    required Widget child,
  }) = _InheritedAnimationStyle;

  /// Returns the [AnimationStyle] corresponding to the nearest ancestor
  /// [DefaultAnimationStyle] widget.
  ///
  /// If no such widget is found, returns an [AnimationStyle] object with
  /// each property set to `null`.
  static AnimationStyle of(BuildContext context, {bool createDependency = true}) {
    final _InheritedAnimationStyle? inherited =
        createDependency
            ? context.dependOnInheritedWidgetOfExactType()
            : context.getInheritedWidgetOfExactType();

    return inherited?.notifier?.value ?? AnimationStyle();
  }

  /// Returns a [ValueListenable] object from the nearest ancestor
  /// [DefaultAnimationStyle] widget.
  ///
  /// This can be useful for responding to animation style updates
  /// without notifying the respective [BuildContext] to rebuild.
  static ValueListenable<AnimationStyle> getNotifier(BuildContext context) {
    final _InheritedAnimationStyle? inheritedWidget = context.getInheritedWidgetOfExactType();
    return inheritedWidget?.notifier ?? const _FallbackAnimationStyleListenable();
  }
}

class _FallbackAnimationStyleListenable implements ValueListenable<AnimationStyle> {
  const _FallbackAnimationStyleListenable();

  @override
  void addListener(VoidCallback listener) {}

  @override
  void removeListener(VoidCallback listener) {}

  @override
  AnimationStyle get value => AnimationStyle();
}

class _InheritedAnimationStyle extends InheritedNotifier<ValueListenable<AnimationStyle>>
    implements DefaultAnimationStyle, InheritedTheme {
  const _InheritedAnimationStyle({super.key, super.notifier, required super.child});

  @override
  Widget wrap(BuildContext context, Widget child) {
    return DefaultAnimationStyle(style: notifier!.value, child: child);
  }
}

class _DefaultAnimationStyle extends StatefulWidget implements DefaultAnimationStyle {
  const _DefaultAnimationStyle({
    super.key,
    required this.style,
    this.mergeWithAncestor = true,
    required this.child,
  });

  final AnimationStyle style;
  final bool mergeWithAncestor;
  final Widget child;

  @override
  State<_DefaultAnimationStyle> createState() => _DefaultAnimationStyleState();
}

class _DefaultAnimationStyleState extends State<_DefaultAnimationStyle> {
  late final ValueNotifier<AnimationStyle> notifier = ValueNotifier<AnimationStyle>(widget.style);
  late ValueListenable<AnimationStyle> ancestor = DefaultAnimationStyle.getNotifier(context);

  @override
  void initState() {
    super.initState();
    ancestor.addListener(_updateStyle);
  }

  @override
  void activate() {
    super.activate();
    final ValueListenable<AnimationStyle> newAncestorNotifier = DefaultAnimationStyle.getNotifier(
      context,
    );
    if (newAncestorNotifier == ancestor) {
      return;
    }
    ancestor.removeListener(_updateStyle);
    ancestor = newAncestorNotifier..addListener(_updateStyle);
  }

  void _updateStyle() {
    final AnimationStyle style = widget.style;
    late final AnimationStyle ancestorStyle = ancestor.value;
    if (widget.mergeWithAncestor && ancestorStyle is! _FallbackAnimationStyleListenable) {
      notifier.value = ancestorStyle.copyWith(
        duration: style.duration,
        curve: style.curve,
        reverseDuration: style.reverseDuration,
        reverseCurve: style.reverseCurve,
      );
    } else {
      notifier.value = style;
    }
  }

  @override
  void dispose() {
    ancestor.removeListener(_updateStyle);
    notifier.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    _updateStyle();
    return DefaultAnimationStyle.notifier(notifier: notifier, child: widget.child);
  }
}
