// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'button.dart';
/// @docImport 'nav_bar.dart';
/// @docImport 'route.dart';
/// @docImport 'tab_scaffold.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';

import 'colors.dart';
import 'theme.dart';

/// Implements a single iOS application page's layout.
///
/// The scaffold lays out the navigation bar on top and the content between or
/// behind the navigation bar.
///
/// When tapping a status bar at the top of the CupertinoPageScaffold, an
/// animation will complete for the current primary [ScrollView], scrolling to
/// the beginning. This is done using the [PrimaryScrollController] that
/// encloses the [ScrollView]. The [ScrollView.primary] flag is used to connect
/// a [ScrollView] to the enclosing [PrimaryScrollController].
///
/// {@tool dartpad}
/// This example shows a [CupertinoPageScaffold] with a [ListView] as a [child].
/// The [CupertinoButton] is connected to a callback that increments a counter.
/// The [backgroundColor] can be changed.
///
/// ** See code in examples/api/lib/cupertino/page_scaffold/cupertino_page_scaffold.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [CupertinoTabScaffold], a similar widget for tabbed applications.
///  * [CupertinoPageRoute], a modal page route that typically hosts a
///    [CupertinoPageScaffold] with support for iOS-style page transitions.
class CupertinoPageScaffold extends StatefulWidget {
  /// Creates a layout for pages with a navigation bar at the top.
  const CupertinoPageScaffold({
    super.key,
    this.navigationBar,
    this.backgroundColor,
    this.resizeToAvoidBottomInset = true,
    required this.child,
  });

  /// The [navigationBar], typically a [CupertinoNavigationBar], is drawn at the
  /// top of the screen.
  ///
  /// If translucent, the main content may slide behind it.
  /// Otherwise, the main content's top margin will be offset by its height.
  ///
  /// The scaffold assumes the navigation bar will account for the [MediaQuery]
  /// top padding, also consume it if the navigation bar is opaque.
  ///
  /// By default [navigationBar] disables text scaling to match the native iOS
  /// behavior. To override such behavior, wrap each of the [navigationBar]'s
  /// components inside a [MediaQuery] with the desired [TextScaler].
  // TODO(xster): document its page transition animation when ready
  final ObstructingPreferredSizeWidget? navigationBar;

  /// Widget to show in the main content area.
  ///
  /// Content can slide under the [navigationBar] when they're translucent.
  /// In that case, the child's [BuildContext]'s [MediaQuery] will have a
  /// top padding indicating the area of obstructing overlap from the
  /// [navigationBar].
  final Widget child;

  /// The color of the widget that underlies the entire scaffold.
  ///
  /// By default uses [CupertinoTheme]'s `scaffoldBackgroundColor` when null.
  final Color? backgroundColor;

  /// Whether the [child] should size itself to avoid the window's bottom inset.
  ///
  /// For example, if there is an onscreen keyboard displayed above the
  /// scaffold, the body can be resized to avoid overlapping the keyboard, which
  /// prevents widgets inside the body from being obscured by the keyboard.
  ///
  /// Defaults to true.
  final bool resizeToAvoidBottomInset;

  @override
  State<CupertinoPageScaffold> createState() => _CupertinoPageScaffoldState();
}

class _CupertinoPageScaffoldState extends State<CupertinoPageScaffold> {
  void _handleStatusBarTap() {
    final ScrollController? primaryScrollController = PrimaryScrollController.maybeOf(context);
    // Only act on the scroll controller if it has any attached scroll positions.
    if (primaryScrollController != null && primaryScrollController.hasClients) {
      primaryScrollController.animateTo(
        0.0,
        // Eyeballed from iOS.
        duration: const Duration(milliseconds: 500),
        curve: Curves.linearToEaseOut,
      );
    }
  }

  @override
  Widget build(BuildContext context) {
    Widget paddedContent = widget.child;

    final Color backgroundColor =
        CupertinoDynamicColor.maybeResolve(widget.backgroundColor, context) ??
        CupertinoTheme.of(context).scaffoldBackgroundColor;

    final MediaQueryData existingMediaQuery = MediaQuery.of(context);
    if (widget.navigationBar != null) {
      // TODO(xster): Use real size after partial layout instead of preferred size.
      // https://github.com/flutter/flutter/issues/12912
      final double topPadding =
          widget.navigationBar!.preferredSize.height + existingMediaQuery.padding.top;

      // Propagate bottom padding and include viewInsets if appropriate
      final double bottomPadding =
          widget.resizeToAvoidBottomInset ? existingMediaQuery.viewInsets.bottom : 0.0;

      final EdgeInsets newViewInsets =
          widget.resizeToAvoidBottomInset
              // The insets are consumed by the scaffolds and no longer exposed to
              // the descendant subtree.
              ? existingMediaQuery.viewInsets.copyWith(bottom: 0.0)
              : existingMediaQuery.viewInsets;

      final bool fullObstruction = widget.navigationBar!.shouldFullyObstruct(context);

      // If navigation bar is opaquely obstructing, directly shift the main content
      // down. If translucent, let main content draw behind navigation bar but hint the
      // obstructed area.
      if (fullObstruction) {
        paddedContent = MediaQuery(
          data: existingMediaQuery
              // If the navigation bar is opaque, the top media query padding is fully consumed by the navigation bar.
              .removePadding(removeTop: true)
              .copyWith(viewInsets: newViewInsets),
          child: Padding(
            padding: EdgeInsets.only(top: topPadding, bottom: bottomPadding),
            child: paddedContent,
          ),
        );
      } else {
        paddedContent = MediaQuery(
          data: existingMediaQuery.copyWith(
            padding: existingMediaQuery.padding.copyWith(top: topPadding),
            viewInsets: newViewInsets,
          ),
          child: Padding(padding: EdgeInsets.only(bottom: bottomPadding), child: paddedContent),
        );
      }
    } else if (widget.resizeToAvoidBottomInset) {
      // If there is no navigation bar, still may need to add padding in order
      // to support resizeToAvoidBottomInset.
      paddedContent = MediaQuery(
        data: existingMediaQuery.copyWith(
          viewInsets: existingMediaQuery.viewInsets.copyWith(bottom: 0),
        ),
        child: Padding(
          padding: EdgeInsets.only(bottom: existingMediaQuery.viewInsets.bottom),
          child: paddedContent,
        ),
      );
    }

    return ScrollNotificationObserver(
      child: DecoratedBox(
        decoration: BoxDecoration(color: backgroundColor),
        child: CupertinoPageScaffoldBackgroundColor(
          color: backgroundColor,
          child: Stack(
            children: <Widget>[
              // The main content being at the bottom is added to the stack first.
              paddedContent,
              if (widget.navigationBar != null)
                Positioned(
                  top: 0.0,
                  left: 0.0,
                  right: 0.0,
                  child: MediaQuery.withNoTextScaling(child: widget.navigationBar!),
                ),
              // Add a touch handler the size of the status bar on top of all contents
              // to handle scroll to top by status bar taps.
              Positioned(
                top: 0.0,
                left: 0.0,
                right: 0.0,
                height: existingMediaQuery.padding.top,
                child: GestureDetector(excludeFromSemantics: true, onTap: _handleStatusBarTap),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

/// [InheritedWidget] indicating what the current scaffold background color is for its children.
///
/// This is used by the [CupertinoNavigationBar] and the [CupertinoSliverNavigationBar] widgets
/// to paint themselves with the parent page scaffold color when no content is scrolled under.
class CupertinoPageScaffoldBackgroundColor extends InheritedWidget {
  /// Constructs a new [CupertinoPageScaffoldBackgroundColor].
  const CupertinoPageScaffoldBackgroundColor({
    required super.child,
    required this.color,
    super.key,
  });

  /// The background color defined in [CupertinoPageScaffold].
  final Color color;

  @override
  bool updateShouldNotify(CupertinoPageScaffoldBackgroundColor oldWidget) {
    return color != oldWidget.color;
  }

  /// Retrieve the [CupertinoPageScaffold] background color from the context.
  static Color? maybeOf(BuildContext context) {
    final CupertinoPageScaffoldBackgroundColor? scaffoldBackgroundColor =
        context.dependOnInheritedWidgetOfExactType<CupertinoPageScaffoldBackgroundColor>();
    return scaffoldBackgroundColor?.color;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(ColorProperty('page scaffold background color', color));
  }
}

/// Widget that has a preferred size and reports whether it fully obstructs
/// widgets behind it.
///
/// Used by [CupertinoPageScaffold] to either shift away fully obstructed content
/// or provide a padding guide to partially obstructed content.
abstract class ObstructingPreferredSizeWidget implements PreferredSizeWidget {
  /// If true, this widget fully obstructs widgets behind it by the specified
  /// size.
  ///
  /// If false, this widget partially obstructs.
  bool shouldFullyObstruct(BuildContext context);
}
