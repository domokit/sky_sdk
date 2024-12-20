// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'navigator.dart';
library;

import 'basic.dart';
import 'framework.dart';
import 'routes.dart';

/// A modal route that replaces the entire screen.
///
/// The [PageRouteBuilder] subclass provides a way to create a [PageRoute] using
/// callbacks rather than by defining a new class via subclassing.
///
/// If `barrierDismissible` is true, then pressing the escape key on the keyboard
/// will cause the current route to be popped with null as the value.
///
/// See also:
///
///  * [Route], which documents the meaning of the `T` generic type argument.
abstract class PageRoute<T> extends ModalRoute<T> {
  /// Creates a modal route that replaces the entire screen.
  PageRoute({
    super.settings,
    super.requestFocus,
    this.fullscreenDialog = false,
    this.allowSnapshotting = true,
    bool barrierDismissible = false,
  }) : _barrierDismissible = barrierDismissible;

  /// {@template flutter.widgets.PageRoute.fullscreenDialog}
  /// Whether this page route is a full-screen dialog.
  ///
  /// In Material and Cupertino, being fullscreen has the effects of making
  /// the app bars have a close button instead of a back button. On
  /// iOS, dialogs transitions animate differently and are also not closeable
  /// with the back swipe gesture.
  /// {@endtemplate}
  final bool fullscreenDialog;

  @override
  final bool allowSnapshotting;

  @override
  bool get opaque => true;

  @override
  bool get barrierDismissible => _barrierDismissible;
  final bool _barrierDismissible;

  @override
  bool canTransitionTo(TransitionRoute<dynamic> nextRoute) => nextRoute is PageRoute;

  @override
  bool canTransitionFrom(TransitionRoute<dynamic> previousRoute) => previousRoute is PageRoute;

  @override
  bool get popGestureEnabled {
    // Fullscreen dialogs aren't dismissible by back swipe.
    return !fullscreenDialog && super.popGestureEnabled;
  }
}

Widget _defaultTransitionsBuilder(
  BuildContext context,
  Animation<double> animation,
  Animation<double> secondaryAnimation,
  Widget child,
) {
  return child;
}

/// A utility class for defining one-off page routes in terms of callbacks.
///
/// Callers must define the [pageBuilder] function which creates the route's
/// primary contents. To add transitions define the [transitionsBuilder] function.
///
/// The `T` generic type argument corresponds to the type argument of the
/// created [Route] objects.
///
/// See also:
///
///  * [Route], which documents the meaning of the `T` generic type argument.
class PageRouteBuilder<T> extends PageRoute<T> {
  /// Creates a route that delegates to builder callbacks.
  PageRouteBuilder({
    super.settings,
    super.requestFocus,
    required this.pageBuilder,
    this.transitionsBuilder = _defaultTransitionsBuilder,
    this.transitionDuration = const Duration(milliseconds: 300),
    this.reverseTransitionDuration = const Duration(milliseconds: 300),
    this.opaque = true,
    this.barrierDismissible = false,
    this.barrierColor,
    this.barrierLabel,
    this.maintainState = true,
    super.fullscreenDialog,
    super.allowSnapshotting = true,
  });

  /// {@template flutter.widgets.pageRouteBuilder.pageBuilder}
  /// Used build the route's primary contents.
  ///
  /// See [ModalRoute.buildPage] for complete definition of the parameters.
  /// {@endtemplate}
  final RoutePageBuilder pageBuilder;

  /// {@template flutter.widgets.pageRouteBuilder.transitionsBuilder}
  /// Used to build the route's transitions.
  ///
  /// See [ModalRoute.buildTransitions] for complete definition of the parameters.
  /// {@endtemplate}
  ///
  /// The default transition is a jump cut (i.e. no animation).
  final RouteTransitionsBuilder transitionsBuilder;

  @override
  final Duration transitionDuration;

  @override
  final Duration reverseTransitionDuration;

  @override
  final bool opaque;

  @override
  final bool barrierDismissible;

  @override
  final Color? barrierColor;

  @override
  final String? barrierLabel;

  @override
  final bool maintainState;

  @override
  Widget buildPage(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
  ) {
    return pageBuilder(context, animation, secondaryAnimation);
  }

  @override
  Widget buildTransitions(
    BuildContext context,
    Animation<double> animation,
    Animation<double> secondaryAnimation,
    Widget child,
  ) {
    return transitionsBuilder(context, animation, secondaryAnimation, child);
  }
}
