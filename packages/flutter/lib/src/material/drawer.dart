// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/gestures.dart';
/// @docImport 'package:flutter/semantics.dart';
///
/// @docImport 'about.dart';
/// @docImport 'app_bar.dart';
/// @docImport 'color_scheme.dart';
/// @docImport 'drawer_header.dart';
/// @docImport 'icon_button.dart';
/// @docImport 'navigation_drawer.dart';
/// @docImport 'scaffold.dart';
library;

import 'package:flutter/gestures.dart' show DragStartBehavior;
import 'package:flutter/widgets.dart';

import 'colors.dart';
import 'debug.dart';
import 'drawer_theme.dart';
import 'list_tile.dart';
import 'list_tile_theme.dart';
import 'material.dart';
import 'material_localizations.dart';
import 'theme.dart';

// Examples can assume:
// late BuildContext context;

/// The possible alignments of a [Drawer].
enum DrawerAlignment {
  /// Denotes that the [Drawer] is at the start side of the [Scaffold].
  ///
  /// This corresponds to the left side when the text direction is left-to-right
  /// and the right side when the text direction is right-to-left.
  start,

  /// Denotes that the [Drawer] is at the end side of the [Scaffold].
  ///
  /// This corresponds to the right side when the text direction is left-to-right
  /// and the left side when the text direction is right-to-left.
  end,
}

// TODO(eseidel): Draw width should vary based on device size:
// https://material.io/design/components/navigation-drawer.html#specs

// Mobile:
// Width = Screen width − 56 dp
// Maximum width: 320dp
// Maximum width applies only when using a left nav. When using a right nav,
// the panel can cover the full width of the screen.

// Desktop/Tablet:
// Maximum width for a left nav is 400dp.
// The right nav can vary depending on content.

const double _kWidth = 304.0;
const double _kEdgeDragWidth = 20.0;
const double _kMinFlingVelocity = 365.0;
const Duration _kBaseSettleDuration = Duration(milliseconds: 246);

/// A Material Design panel that slides in horizontally from the edge of a
/// [Scaffold] to show navigation links in an application.
///
/// There is a Material 3 version of this component, [NavigationDrawer],
/// that's preferred for applications that are configured for Material 3
/// (see [ThemeData.useMaterial3]).
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=WRj86iHihgY}
///
/// Drawers are typically used with the [Scaffold.drawer] property. The child of
/// the drawer is usually a [ListView] whose first child is a [DrawerHeader]
/// that displays status information about the current user. The remaining
/// drawer children are often constructed with [ListTile]s, often concluding
/// with an [AboutListTile].
///
/// The [AppBar] automatically displays an appropriate [IconButton] to show the
/// [Drawer] when a [Drawer] is available in the [Scaffold]. The [Scaffold]
/// automatically handles the edge-swipe gesture to show the drawer.
///
/// {@animation 350 622 https://flutter.github.io/assets-for-api-docs/assets/material/drawer.mp4}
///
/// ## Updating to [NavigationDrawer]
///
/// There is a Material 3 version of this component, [NavigationDrawer],
/// that's preferred for applications that are configured for Material 3
/// (see [ThemeData.useMaterial3]). The [NavigationDrawer] widget's visual
/// are a little bit different, see the Material 3 spec at
/// <https://m3.material.io/components/navigation-drawer/overview> for
/// more details. While the [Drawer] widget can have only one child, the
/// [NavigationDrawer] widget can have a list of widgets, which typically contains
/// [NavigationDrawerDestination] widgets and/or customized widgets like headlines
/// and dividers.
///
/// {@tool dartpad}
/// This example shows how to create a [Scaffold] that contains an [AppBar] and
/// a [Drawer]. A user taps the "menu" icon in the [AppBar] to open the
/// [Drawer]. The [Drawer] displays four items: A header and three menu items.
/// The [Drawer] displays the four items using a [ListView], which allows the
/// user to scroll through the items if need be.
///
/// ** See code in examples/api/lib/material/drawer/drawer.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows how to migrate the above [Drawer] to a [NavigationDrawer].
///
/// ** See code in examples/api/lib/material/navigation_drawer/navigation_drawer.0.dart **
/// {@end-tool}
///
/// An open drawer may be closed with a swipe to close gesture, pressing the
/// escape key, by tapping the scrim, or by calling pop route function such as
/// [Navigator.pop]. For example a drawer item might close the drawer when tapped:
///
/// ```dart
/// ListTile(
///   leading: const Icon(Icons.change_history),
///   title: const Text('Change history'),
///   onTap: () {
///     // change app state...
///     Navigator.pop(context); // close the drawer
///   },
/// );
/// ```
///
/// See also:
///
///  * [Scaffold.drawer], where one specifies a [Drawer] so that it can be
///    shown.
///  * [Scaffold.of], to obtain the current [ScaffoldState], which manages the
///    display and animation of the drawer.
///  * [ScaffoldState.openDrawer], which displays its [Drawer], if any.
///  * <https://material.io/design/components/navigation-drawer.html>
class Drawer extends StatelessWidget {
  /// Creates a Material Design drawer.
  ///
  /// Typically used in the [Scaffold.drawer] property.
  ///
  /// The [elevation] must be non-negative.
  const Drawer({
    super.key,
    this.backgroundColor,
    this.elevation,
    this.shadowColor,
    this.surfaceTintColor,
    this.shape,
    this.width,
    this.child,
    this.semanticLabel,
    this.clipBehavior,
  }) : assert(elevation == null || elevation >= 0.0);

  /// Sets the color of the [Material] that holds all of the [Drawer]'s
  /// contents.
  ///
  /// If this is null, then [DrawerThemeData.backgroundColor] is used. If that
  /// is also null, then it falls back to [Material]'s default.
  final Color? backgroundColor;

  /// The z-coordinate at which to place this drawer relative to its parent.
  ///
  /// This controls the size of the shadow below the drawer.
  ///
  /// If this is null, then [DrawerThemeData.elevation] is used. If that
  /// is also null, then it defaults to 16.0.
  final double? elevation;

  /// The color used to paint a drop shadow under the drawer's [Material],
  /// which reflects the drawer's [elevation].
  ///
  /// If null and [ThemeData.useMaterial3] is true then no drop shadow will
  /// be rendered.
  ///
  /// If null and [ThemeData.useMaterial3] is false then it will default to
  /// [ThemeData.shadowColor].
  ///
  /// See also:
  ///   * [Material.shadowColor], which describes how the drop shadow is painted.
  ///   * [elevation], which affects how the drop shadow is painted.
  ///   * [surfaceTintColor], which can be used to indicate elevation through
  ///     tinting the background color.
  final Color? shadowColor;

  /// The color used as a surface tint overlay on the drawer's background color,
  /// which reflects the drawer's [elevation].
  ///
  /// This is not recommended for use. [Material 3 spec](https://m3.material.io/styles/color/the-color-system/color-roles)
  /// introduced a set of tone-based surfaces and surface containers in its [ColorScheme],
  /// which provide more flexibility. The intention is to eventually remove surface tint color from
  /// the framework.
  ///
  /// To disable this feature, set [surfaceTintColor] to [Colors.transparent].
  ///
  /// Defaults to [Colors.transparent].
  ///
  /// See also:
  ///   * [Material.surfaceTintColor], which describes how the surface tint will
  ///     be applied to the background color of the drawer.
  ///   * [elevation], which affects the opacity of the surface tint.
  ///   * [shadowColor], which can be used to indicate elevation through
  ///     a drop shadow.
  final Color? surfaceTintColor;

  /// The shape of the drawer.
  ///
  /// Defines the drawer's [Material.shape].
  ///
  /// If this is null, then [DrawerThemeData.shape] is used. If that
  /// is also null, then it falls back to [Material]'s default.
  final ShapeBorder? shape;

  /// The width of the drawer.
  ///
  /// If this is null, then [DrawerThemeData.width] is used. If that is also
  /// null, then it falls back to the Material spec's default (304.0).
  final double? width;

  /// The widget below this widget in the tree.
  ///
  /// Typically a [SliverList].
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// The semantic label of the drawer used by accessibility frameworks to
  /// announce screen transitions when the drawer is opened and closed.
  ///
  /// If this label is not provided, it will default to
  /// [MaterialLocalizations.drawerLabel].
  ///
  /// See also:
  ///
  ///  * [SemanticsConfiguration.namesRoute], for a description of how this
  ///    value is used.
  final String? semanticLabel;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// The [clipBehavior] argument specifies how to clip the drawer's [shape].
  ///
  /// If the drawer has a [shape], it defaults to [Clip.hardEdge]. Otherwise,
  /// defaults to [Clip.none].
  final Clip? clipBehavior;

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMaterialLocalizations(context));
    final DrawerThemeData drawerTheme = DrawerTheme.of(context);
    String? label = semanticLabel;
    switch (Theme.of(context).platform) {
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        break;
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        label = semanticLabel ?? MaterialLocalizations.of(context).drawerLabel;
    }
    final bool useMaterial3 = Theme.of(context).useMaterial3;
    final bool isDrawerStart = DrawerController.maybeOf(context)?.alignment != DrawerAlignment.end;
    final DrawerThemeData defaults =
        useMaterial3 ? _DrawerDefaultsM3(context) : _DrawerDefaultsM2(context);
    final ShapeBorder? effectiveShape =
        shape ??
        (isDrawerStart
            ? (drawerTheme.shape ?? defaults.shape)
            : (drawerTheme.endShape ?? defaults.endShape));
    return Semantics(
      scopesRoute: true,
      namesRoute: true,
      explicitChildNodes: true,
      label: label,
      child: ConstrainedBox(
        constraints: BoxConstraints.expand(width: width ?? drawerTheme.width ?? _kWidth),
        child: Material(
          color: backgroundColor ?? drawerTheme.backgroundColor ?? defaults.backgroundColor,
          elevation: elevation ?? drawerTheme.elevation ?? defaults.elevation!,
          shadowColor: shadowColor ?? drawerTheme.shadowColor ?? defaults.shadowColor,
          surfaceTintColor:
              surfaceTintColor ?? drawerTheme.surfaceTintColor ?? defaults.surfaceTintColor,
          shape: effectiveShape,
          clipBehavior:
              effectiveShape != null
                  ? (clipBehavior ?? drawerTheme.clipBehavior ?? defaults.clipBehavior!)
                  : Clip.none,
          child: child,
        ),
      ),
    );
  }
}

/// Signature for the callback that's called when a [DrawerController] is
/// opened or closed.
typedef DrawerCallback = void Function(bool isOpened);

class _DrawerControllerScope extends InheritedWidget {
  const _DrawerControllerScope({required this.controller, required super.child});

  final DrawerController controller;

  @override
  bool updateShouldNotify(_DrawerControllerScope old) {
    return controller != old.controller;
  }
}

/// Provides interactive behavior for [Drawer] widgets.
///
/// Rarely used directly. Drawer controllers are typically created automatically
/// by [Scaffold] widgets.
///
/// The drawer controller provides the ability to open and close a drawer, either
/// via an animation or via user interaction. When closed, the drawer collapses
/// to a translucent gesture detector that can be used to listen for edge
/// swipes.
///
/// See also:
///
///  * [Drawer], a container with the default width of a drawer.
///  * [Scaffold.drawer], the [Scaffold] slot for showing a drawer.
class DrawerController extends StatefulWidget {
  /// Creates a controller for a [Drawer].
  ///
  /// Rarely used directly.
  ///
  /// The [child] argument is typically a [Drawer].
  const DrawerController({
    GlobalKey? key,
    required this.child,
    required this.alignment,
    this.isDrawerOpen = false,
    this.drawerCallback,
    this.dragStartBehavior = DragStartBehavior.start,
    this.scrimColor,
    this.edgeDragWidth,
    this.enableOpenDragGesture = true,
  }) : super(key: key);

  /// The widget below this widget in the tree.
  ///
  /// Typically a [Drawer].
  final Widget child;

  /// The alignment of the [Drawer].
  ///
  /// This controls the direction in which the user should swipe to open and
  /// close the drawer.
  final DrawerAlignment alignment;

  /// Optional callback that is called when a [Drawer] is opened or closed.
  final DrawerCallback? drawerCallback;

  /// {@template flutter.material.DrawerController.dragStartBehavior}
  /// Determines the way that drag start behavior is handled.
  ///
  /// If set to [DragStartBehavior.start], the drag behavior used for opening
  /// and closing a drawer will begin at the position where the drag gesture won
  /// the arena. If set to [DragStartBehavior.down] it will begin at the position
  /// where a down event is first detected.
  ///
  /// In general, setting this to [DragStartBehavior.start] will make drag
  /// animation smoother and setting it to [DragStartBehavior.down] will make
  /// drag behavior feel slightly more reactive.
  ///
  /// By default, the drag start behavior is [DragStartBehavior.start].
  ///
  /// See also:
  ///
  ///  * [DragGestureRecognizer.dragStartBehavior], which gives an example for
  ///    the different behaviors.
  ///
  /// {@endtemplate}
  final DragStartBehavior dragStartBehavior;

  /// The color to use for the scrim that obscures the underlying content while
  /// a drawer is open.
  ///
  /// If this is null, then [DrawerThemeData.scrimColor] is used. If that
  /// is also null, then it defaults to [Colors.black54].
  final Color? scrimColor;

  /// Determines if the [Drawer] can be opened with a drag gesture.
  ///
  /// By default, the drag gesture is enabled.
  final bool enableOpenDragGesture;

  /// The width of the area within which a horizontal swipe will open the
  /// drawer.
  ///
  /// By default, the value used is 20.0 added to the padding edge of
  /// `MediaQuery.paddingOf(context)` that corresponds to [alignment].
  /// This ensures that the drag area for notched devices is not obscured. For
  /// example, if [alignment] is set to [DrawerAlignment.start] and
  /// `TextDirection.of(context)` is set to [TextDirection.ltr],
  /// 20.0 will be added to `MediaQuery.paddingOf(context).left`.
  final double? edgeDragWidth;

  /// Whether or not the drawer is opened or closed.
  ///
  /// This parameter is primarily used by the state restoration framework
  /// to restore the drawer's animation controller to the open or closed state
  /// depending on what was last saved to the target platform before the
  /// application was killed.
  final bool isDrawerOpen;

  /// The closest instance of [DrawerController] that encloses the given
  /// context, or null if none is found.
  ///
  /// {@tool snippet} Typical usage is as follows:
  ///
  /// ```dart
  /// DrawerController? controller = DrawerController.maybeOf(context);
  /// ```
  /// {@end-tool}
  ///
  /// Calling this method will create a dependency on the closest
  /// [DrawerController] in the [context], if there is one.
  ///
  /// See also:
  ///
  /// * [DrawerController.of], which is similar to this method, but asserts
  ///   if no [DrawerController] ancestor is found.
  static DrawerController? maybeOf(BuildContext context) {
    return context.dependOnInheritedWidgetOfExactType<_DrawerControllerScope>()?.controller;
  }

  /// The closest instance of [DrawerController] that encloses the given
  /// context.
  ///
  /// If no instance is found, this method will assert in debug mode and throw
  /// an exception in release mode.
  ///
  /// Calling this method will create a dependency on the closest
  /// [DrawerController] in the [context].
  ///
  /// {@tool snippet} Typical usage is as follows:
  ///
  /// ```dart
  /// DrawerController controller = DrawerController.of(context);
  /// ```
  /// {@end-tool}
  static DrawerController of(BuildContext context) {
    final DrawerController? controller = maybeOf(context);
    assert(() {
      if (controller == null) {
        throw FlutterError(
          'DrawerController.of() was called with a context that does not '
          'contain a DrawerController widget.\n'
          'No DrawerController widget ancestor could be found starting from '
          'the context that was passed to DrawerController.of(). This can '
          'happen because you are using a widget that looks for a DrawerController '
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
  DrawerControllerState createState() => DrawerControllerState();
}

/// State for a [DrawerController].
///
/// Typically used by a [Scaffold] to [open] and [close] the drawer.
class DrawerControllerState extends State<DrawerController> with SingleTickerProviderStateMixin {
  @protected
  @override
  void initState() {
    super.initState();
    _controller = AnimationController(
      value: widget.isDrawerOpen ? 1.0 : 0.0,
      duration: _kBaseSettleDuration,
      vsync: this,
    );
    _controller
      ..addListener(_animationChanged)
      ..addStatusListener(_animationStatusChanged);
  }

  @protected
  @override
  void dispose() {
    _historyEntry?.remove();
    _controller.dispose();
    _focusScopeNode.dispose();
    super.dispose();
  }

  @protected
  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    _scrimColorTween = _buildScrimColorTween();
  }

  @protected
  @override
  void didUpdateWidget(DrawerController oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.scrimColor != oldWidget.scrimColor) {
      _scrimColorTween = _buildScrimColorTween();
    }

    if (_controller.status.isAnimating) {
      return; // Don't snap the drawer open or shut while the user is dragging.
    }
    if (widget.isDrawerOpen != oldWidget.isDrawerOpen) {
      _controller.value = widget.isDrawerOpen ? 1.0 : 0.0;
    }
  }

  void _animationChanged() {
    setState(() {
      // The animation controller's state is our build state, and it changed already.
    });
  }

  LocalHistoryEntry? _historyEntry;
  final FocusScopeNode _focusScopeNode = FocusScopeNode();

  void _ensureHistoryEntry() {
    if (_historyEntry == null) {
      final ModalRoute<dynamic>? route = ModalRoute.of(context);
      if (route != null) {
        _historyEntry = LocalHistoryEntry(
          onRemove: _handleHistoryEntryRemoved,
          impliesAppBarDismissal: false,
        );
        route.addLocalHistoryEntry(_historyEntry!);
        FocusScope.of(context).setFirstFocus(_focusScopeNode);
      }
    }
  }

  void _animationStatusChanged(AnimationStatus status) {
    switch (status) {
      case AnimationStatus.forward:
        _ensureHistoryEntry();
      case AnimationStatus.reverse:
        _historyEntry?.remove();
        _historyEntry = null;
      case AnimationStatus.dismissed:
      case AnimationStatus.completed:
        break;
    }
  }

  void _handleHistoryEntryRemoved() {
    _historyEntry = null;
    close();
  }

  late AnimationController _controller;

  void _handleDragDown(DragDownDetails details) {
    _controller.stop();
    _ensureHistoryEntry();
  }

  void _handleDragCancel() {
    if (_controller.isDismissed || _controller.isAnimating) {
      return;
    }
    if (_controller.value < 0.5) {
      close();
    } else {
      open();
    }
  }

  final GlobalKey _drawerKey = GlobalKey();

  double get _width {
    final RenderBox? box = _drawerKey.currentContext?.findRenderObject() as RenderBox?;
    // return _kWidth if drawer not being shown currently
    return box?.size.width ?? _kWidth;
  }

  bool _previouslyOpened = false;

  int get _directionFactor {
    return switch ((Directionality.of(context), widget.alignment)) {
      (TextDirection.rtl, DrawerAlignment.start) => -1,
      (TextDirection.rtl, DrawerAlignment.end) => 1,
      (TextDirection.ltr, DrawerAlignment.start) => 1,
      (TextDirection.ltr, DrawerAlignment.end) => -1,
    };
  }

  void _move(DragUpdateDetails details) {
    _controller.value += details.primaryDelta! / _width * _directionFactor;

    final bool opened = _controller.value > 0.5;
    if (opened != _previouslyOpened && widget.drawerCallback != null) {
      widget.drawerCallback!(opened);
    }
    _previouslyOpened = opened;
  }

  void _settle(DragEndDetails details) {
    if (_controller.isDismissed) {
      return;
    }
    final double xVelocity = details.velocity.pixelsPerSecond.dx;
    if (xVelocity.abs() >= _kMinFlingVelocity) {
      final double visualVelocity = xVelocity / _width * _directionFactor;

      _controller.fling(velocity: visualVelocity);
      widget.drawerCallback?.call(visualVelocity > 0.0);
    } else if (_controller.value < 0.5) {
      close();
    } else {
      open();
    }
  }

  /// Starts an animation to open the drawer.
  ///
  /// Typically called by [ScaffoldState.openDrawer].
  void open() {
    _controller.fling();
    widget.drawerCallback?.call(true);
  }

  /// Starts an animation to close the drawer.
  void close() {
    _controller.fling(velocity: -1.0);
    widget.drawerCallback?.call(false);
  }

  late ColorTween _scrimColorTween;
  final GlobalKey _gestureDetectorKey = GlobalKey();

  ColorTween _buildScrimColorTween() {
    return ColorTween(
      begin: Colors.transparent,
      end: widget.scrimColor ?? DrawerTheme.of(context).scrimColor ?? Colors.black54,
    );
  }

  AlignmentDirectional get _drawerOuterAlignment => switch (widget.alignment) {
    DrawerAlignment.start => AlignmentDirectional.centerStart,
    DrawerAlignment.end => AlignmentDirectional.centerEnd,
  };

  AlignmentDirectional get _drawerInnerAlignment => switch (widget.alignment) {
    DrawerAlignment.start => AlignmentDirectional.centerEnd,
    DrawerAlignment.end => AlignmentDirectional.centerStart,
  };

  Widget _buildDrawer(BuildContext context) {
    final bool isDesktop = switch (Theme.of(context).platform) {
      TargetPlatform.android || TargetPlatform.iOS || TargetPlatform.fuchsia => false,
      TargetPlatform.macOS || TargetPlatform.linux || TargetPlatform.windows => true,
    };

    final double dragAreaWidth =
        widget.edgeDragWidth ??
        _kEdgeDragWidth +
            switch ((widget.alignment, Directionality.of(context))) {
              (DrawerAlignment.start, TextDirection.ltr) => MediaQuery.paddingOf(context).left,
              (DrawerAlignment.start, TextDirection.rtl) => MediaQuery.paddingOf(context).right,
              (DrawerAlignment.end, TextDirection.rtl) => MediaQuery.paddingOf(context).left,
              (DrawerAlignment.end, TextDirection.ltr) => MediaQuery.paddingOf(context).right,
            };

    if (_controller.isDismissed) {
      if (widget.enableOpenDragGesture && !isDesktop) {
        return Align(
          alignment: _drawerOuterAlignment,
          child: GestureDetector(
            key: _gestureDetectorKey,
            onHorizontalDragUpdate: _move,
            onHorizontalDragEnd: _settle,
            behavior: HitTestBehavior.translucent,
            excludeFromSemantics: true,
            dragStartBehavior: widget.dragStartBehavior,
            child: LimitedBox(
              maxHeight: 0.0,
              child: SizedBox(width: dragAreaWidth, height: double.infinity),
            ),
          ),
        );
      } else {
        return const SizedBox.shrink();
      }
    } else {
      final bool platformHasBackButton;
      switch (Theme.of(context).platform) {
        case TargetPlatform.android:
          platformHasBackButton = true;
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          platformHasBackButton = false;
      }

      Widget drawerScrim = const LimitedBox(
        maxWidth: 0.0,
        maxHeight: 0.0,
        child: SizedBox.expand(),
      );
      if (_scrimColorTween.evaluate(_controller) case final Color color) {
        drawerScrim = ColoredBox(color: color, child: drawerScrim);
      }

      final Widget child = _DrawerControllerScope(
        controller: widget,
        child: RepaintBoundary(
          child: Stack(
            children: <Widget>[
              BlockSemantics(
                child: ExcludeSemantics(
                  // On Android, the back button is used to dismiss a modal.
                  excluding: platformHasBackButton,
                  child: GestureDetector(
                    onTap: close,
                    child: Semantics(
                      label: MaterialLocalizations.of(context).modalBarrierDismissLabel,
                      child: drawerScrim,
                    ),
                  ),
                ),
              ),
              Align(
                alignment: _drawerOuterAlignment,
                child: Align(
                  alignment: _drawerInnerAlignment,
                  widthFactor: _controller.value,
                  child: RepaintBoundary(
                    child: FocusScope(key: _drawerKey, node: _focusScopeNode, child: widget.child),
                  ),
                ),
              ),
            ],
          ),
        ),
      );

      if (isDesktop) {
        return child;
      }

      return GestureDetector(
        key: _gestureDetectorKey,
        onHorizontalDragDown: _handleDragDown,
        onHorizontalDragUpdate: _move,
        onHorizontalDragEnd: _settle,
        onHorizontalDragCancel: _handleDragCancel,
        excludeFromSemantics: true,
        dragStartBehavior: widget.dragStartBehavior,
        child: child,
      );
    }
  }

  @protected
  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMaterialLocalizations(context));
    return ListTileTheme.merge(style: ListTileStyle.drawer, child: _buildDrawer(context));
  }
}

class _DrawerDefaultsM2 extends DrawerThemeData {
  const _DrawerDefaultsM2(this.context) : super(elevation: 16.0, clipBehavior: Clip.hardEdge);

  final BuildContext context;

  @override
  Color? get shadowColor => Theme.of(context).shadowColor;
}

// BEGIN GENERATED TOKEN PROPERTIES - Drawer

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _DrawerDefaultsM3 extends DrawerThemeData {
  _DrawerDefaultsM3(this.context)
      : super(
          elevation: 1.0,
          clipBehavior: Clip.hardEdge,
        );

  final BuildContext context;
  late final TextDirection direction = Directionality.of(context);

  @override
  Color? get backgroundColor => Theme.of(context).colorScheme.surfaceContainerLow;

  @override
  Color? get surfaceTintColor => Colors.transparent;

  @override
  Color? get shadowColor => Colors.transparent;

  // There isn't currently a token for this value, but it is shown in the spec,
  // so hard coding here for now.
  @override
  ShapeBorder? get shape => RoundedRectangleBorder(
    borderRadius: const BorderRadiusDirectional.horizontal(
      end: Radius.circular(16.0),
    ).resolve(direction),
  );

  // There isn't currently a token for this value, but it is shown in the spec,
  // so hard coding here for now.
  @override
  ShapeBorder? get endShape => RoundedRectangleBorder(
    borderRadius: const BorderRadiusDirectional.horizontal(
      start: Radius.circular(16.0),
    ).resolve(direction),
  );
}
// dart format on

// END GENERATED TOKEN PROPERTIES - Drawer
