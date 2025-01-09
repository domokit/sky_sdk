// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
/// @docImport 'package:flutter/rendering.dart';
/// @docImport 'package:flutter_test/flutter_test.dart';
///
/// @docImport 'actions.dart';
/// @docImport 'focus_manager.dart';
/// @docImport 'scroll_view.dart';
/// @docImport 'scrollable.dart';
/// @docImport 'scrollable_helpers.dart';
/// @docImport 'shortcuts.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/painting.dart';

import 'framework.dart';
import 'scroll_configuration.dart';
import 'scroll_controller.dart';

const Set<TargetPlatform> _kMobilePlatforms = <TargetPlatform>{
  TargetPlatform.android,
  TargetPlatform.iOS,
  TargetPlatform.fuchsia,
};

/// Associates a [ScrollController] with a subtree.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=33_0ABjFJUU}
///
/// When a [ScrollView] has [ScrollView.primary] set to true, the [ScrollView]
/// uses [of] to inherit the [PrimaryScrollController] associated with its
/// subtree.
///
/// A ScrollView that doesn't have a controller or the primary flag set will
/// inherit the PrimaryScrollController, if [shouldInherit] allows it. By
/// default [shouldInherit] is true for mobile platforms when the ScrollView has
/// a scroll direction of [Axis.vertical]. This automatic inheritance can be
/// configured with [automaticallyInheritForPlatforms] and [scrollDirection].
///
/// Inheriting this ScrollController can provide default behavior for scroll
/// views in a subtree. For example, the [Scaffold] uses this mechanism to
/// implement the scroll-to-top gesture on iOS.
///
/// Another default behavior handled by the PrimaryScrollController is default
/// [ScrollAction]s. If a ScrollAction is not handled by an otherwise focused
/// part of the application, the ScrollAction will be evaluated using the scroll
/// view associated with a PrimaryScrollController, for example, when executing
/// [Shortcuts] key events like page up and down.
///
/// See also:
///   * [ScrollAction], an [Action] that scrolls the [Scrollable] that encloses
///     the current [primaryFocus] or is attached to the PrimaryScrollController.
///   * [Shortcuts], a widget that establishes a [ShortcutManager] to be used
///     by its descendants when invoking an [Action] via a keyboard key
///     combination that maps to an [Intent].
class PrimaryScrollController extends InheritedWidget {
  /// Creates a widget that associates a [ScrollController] with a subtree.
  const PrimaryScrollController({
    super.key,
    required ScrollController this.controller,
    this.automaticallyInheritForPlatforms = _kMobilePlatforms,
    this.scrollDirection = Axis.vertical,
    required super.child,
  });

  /// Creates a subtree without an associated [ScrollController].
  const PrimaryScrollController.none({super.key, required super.child})
    : automaticallyInheritForPlatforms = const <TargetPlatform>{},
      scrollDirection = null,
      controller = null;

  /// The [ScrollController] associated with the subtree.
  ///
  /// See also:
  ///
  ///  * [ScrollView.controller], which discusses the purpose of specifying a
  ///    scroll controller.
  final ScrollController? controller;

  /// The [Axis] this controller is configured for [ScrollView]s to
  /// automatically inherit.
  ///
  /// Used in conjunction with [automaticallyInheritForPlatforms]. If the
  /// current [TargetPlatform] is not included in
  /// [automaticallyInheritForPlatforms], this is ignored.
  ///
  /// When null, no [ScrollView] in any Axis will automatically inherit this
  /// controller. This is dissimilar to [PrimaryScrollController.none]. When a
  /// PrimaryScrollController is inherited, ScrollView will insert
  /// PrimaryScrollController.none into the tree to prevent further descendant
  /// ScrollViews from inheriting the current PrimaryScrollController.
  ///
  /// For the direction in which active scrolling may be occurring, see
  /// [ScrollDirection].
  ///
  /// Defaults to [Axis.vertical].
  final Axis? scrollDirection;

  /// The [TargetPlatform]s this controller is configured for [ScrollView]s to
  /// automatically inherit.
  ///
  /// Used in conjunction with [scrollDirection]. If the [Axis] provided to
  /// [shouldInherit] is not [scrollDirection], this is ignored.
  ///
  /// When empty, no ScrollView in any Axis will automatically inherit this
  /// controller. Defaults to [TargetPlatformVariant.mobile].
  final Set<TargetPlatform> automaticallyInheritForPlatforms;

  /// Returns true if this PrimaryScrollController is configured to be
  /// automatically inherited for the current [TargetPlatform] and the given
  /// [Axis].
  ///
  /// This method is typically not called directly. [ScrollView] will call this
  /// method if it has not been provided a [ScrollController] and
  /// [ScrollView.primary] is unset.
  ///
  /// If a ScrollController has already been provided to
  /// [ScrollView.controller], or [ScrollView.primary] is set, this is method is
  /// not called by ScrollView as it will have determined whether or not to
  /// inherit the PrimaryScrollController.
  static bool shouldInherit(BuildContext context, Axis scrollDirection) {
    final PrimaryScrollController? result =
        context.findAncestorWidgetOfExactType<PrimaryScrollController>();
    if (result == null) {
      return false;
    }

    final TargetPlatform platform = ScrollConfiguration.of(context).getPlatform(context);
    if (result.automaticallyInheritForPlatforms.contains(platform)) {
      return result.scrollDirection == scrollDirection;
    }
    return false;
  }

  /// Returns the [ScrollController] most closely associated with the given
  /// context.
  ///
  /// Returns null if there is no [ScrollController] associated with the given
  /// context.
  ///
  /// Calling this method will create a dependency on the closest
  /// [PrimaryScrollController] in the [context], if there is one.
  ///
  /// See also:
  ///
  /// * [PrimaryScrollController.maybeOf], which is similar to this method, but
  ///   asserts if no [PrimaryScrollController] ancestor is found.
  static ScrollController? maybeOf(BuildContext context) {
    final PrimaryScrollController? result =
        context.dependOnInheritedWidgetOfExactType<PrimaryScrollController>();
    return result?.controller;
  }

  /// Returns the [ScrollController] most closely associated with the given
  /// context.
  ///
  /// If no ancestor is found, this method will assert in debug mode, and throw
  /// an exception in release mode.
  ///
  /// Calling this method will create a dependency on the closest
  /// [PrimaryScrollController] in the [context].
  ///
  /// See also:
  ///
  /// * [PrimaryScrollController.maybeOf], which is similar to this method, but
  ///   returns null if no [PrimaryScrollController] ancestor is found.
  static ScrollController of(BuildContext context) {
    final ScrollController? controller = maybeOf(context);
    assert(() {
      if (controller == null) {
        throw FlutterError(
          'PrimaryScrollController.of() was called with a context that does not contain a '
          'PrimaryScrollController widget.\n'
          'No PrimaryScrollController widget ancestor could be found starting from the '
          'context that was passed to PrimaryScrollController.of(). This can happen '
          'because you are using a widget that looks for a PrimaryScrollController '
          'ancestor, but no such ancestor exists.\n'
          'The context used was:\n'
          '  $context',
        );
      }
      return true;
    }());
    return controller!;
  }

  @override
  bool updateShouldNotify(PrimaryScrollController oldWidget) => controller != oldWidget.controller;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      DiagnosticsProperty<ScrollController>(
        'controller',
        controller,
        ifNull: 'no controller',
        showName: false,
      ),
    );
  }
}
