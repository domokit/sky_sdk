// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'framework.dart';

// Examples can assume:
// TooltipThemeData data = const TooltipThemeData();

/// An [InheritedWidget] that defines visual properties like colors
/// and text styles, which the [child]'s subtree depends on.
///
/// The [wrap] method is used by [captureAll] and [CapturedThemes.wrap] to
/// construct a widget that will wrap a child in all of the inherited themes
/// which are present in a specified part of the widget tree.
///
/// A widget that's shown in a different context from the one it's built in,
/// like the contents of a new route or an overlay, will be able to see the
/// ancestor inherited themes of the context it was built in.
///
/// {@tool dartpad}
/// This example demonstrates how `InheritedTheme.capture()` can be used
/// to wrap the contents of a new route with the inherited themes that
/// are present when the route was built - but are not present when route
/// is actually shown.
///
/// If the same code is run without `InheritedTheme.capture(), the
/// new route's Text widget will inherit the "something must be wrong"
/// fallback text style, rather than the default text style defined in MyApp.
///
/// ** See code in examples/api/lib/widgets/inherited_theme/inherited_theme.0.dart **
/// {@end-tool}
abstract class InheritedTheme extends InheritedWidget {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.

  const InheritedTheme({super.key, required super.child});

  /// Return a copy of this inherited theme with the specified [child].
  ///
  /// This implementation for [TooltipTheme] is typical:
  ///
  /// ```dart
  /// Widget wrap(BuildContext context, Widget child) {
  ///   return TooltipTheme(data: data, child: child);
  /// }
  /// ```
  Widget wrap(BuildContext context, Widget child);

  /// Returns a widget that will [wrap] `child` in all of the inherited themes
  /// which are present between `context` and the specified `to`
  /// [BuildContext].
  ///
  /// The `to` context must be an ancestor of `context`. If `to` is not
  /// specified, all inherited themes up to the root of the widget tree are
  /// captured.
  ///
  /// After calling this method, the themes present between `context` and `to`
  /// are frozen for the provided `child`. If the themes (or their theme data)
  /// change in the original subtree, those changes will not be visible to
  /// the wrapped `child` - unless this method is called again to re-wrap the
  /// child.
  static Widget captureAll(BuildContext context, Widget child, {BuildContext? to}) {
    return capture(from: context, to: to).wrap(child);
  }

  /// Returns a [CapturedThemes] object that includes all the [InheritedTheme]s
  /// between the given `from` and `to` [BuildContext]s.
  ///
  /// The `to` context must be an ancestor of the `from` context. If `to` is
  /// null, all ancestor inherited themes of `from` up to the root of the
  /// widget tree are captured.
  ///
  /// After calling this method, the themes present between `from` and `to` are
  /// frozen in the returned [CapturedThemes] object. If the themes (or their
  /// theme data) change in the original subtree, those changes will not be
  /// applied to the themes captured in the [CapturedThemes] object - unless
  /// this method is called again to re-capture the updated themes.
  ///
  /// To wrap a [Widget] in the captured themes, call [CapturedThemes.wrap].
  ///
  /// This method can be expensive if there are many widgets between `from` and
  /// `to` (it walks the element tree between those nodes).
  static CapturedThemes capture({required BuildContext from, required BuildContext? to}) {
    if (from == to) {
      // Nothing to capture.
      return CapturedThemes._(const <InheritedTheme>[]);
    }

    final List<InheritedTheme> themes = <InheritedTheme>[];
    final Set<Type> themeTypes = <Type>{};
    late bool debugDidFindAncestor;
    assert(() {
      debugDidFindAncestor = to == null;
      return true;
    }());
    from.visitAncestorElements((Element ancestor) {
      if (ancestor == to) {
        assert(() {
          debugDidFindAncestor = true;
          return true;
        }());
        return false;
      }
      if (ancestor case InheritedElement(widget: final InheritedTheme theme)) {
        final Type themeType = theme.runtimeType;
        // Only remember the first theme of any type. This assumes
        // that inherited themes completely shadow ancestors of the
        // same type.
        if (!themeTypes.contains(themeType)) {
          themeTypes.add(themeType);
          themes.add(theme);
        }
      }
      return true;
    });

    assert(
      debugDidFindAncestor,
      'The provided `to` context must be an ancestor of the `from` context.',
    );
    return CapturedThemes._(themes);
  }
}

/// Stores a list of captured [InheritedTheme]s that can be wrapped around a
/// child [Widget].
///
/// Used as return type by [InheritedTheme.capture].
class CapturedThemes {
  CapturedThemes._(this._themes);

  final List<InheritedTheme> _themes;

  /// Wraps a `child` [Widget] in the [InheritedTheme]s captured in this object.
  Widget wrap(Widget child) {
    return _CaptureAll(themes: _themes, child: child);
  }
}

class _CaptureAll extends StatelessWidget {
  const _CaptureAll({required this.themes, required this.child});

  final List<InheritedTheme> themes;
  final Widget child;

  @override
  Widget build(BuildContext context) {
    Widget wrappedChild = child;
    for (final InheritedTheme theme in themes) {
      wrappedChild = theme.wrap(context, wrappedChild);
    }
    return wrappedChild;
  }
}
