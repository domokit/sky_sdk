// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';

import 'actions.dart';
import 'basic.dart';
import 'container.dart';
import 'display_feature_sub_screen.dart';
import 'focus_manager.dart';
import 'focus_scope.dart';
import 'focus_traversal.dart';
import 'framework.dart';
import 'media_query.dart';
import 'overlay.dart';
import 'scroll_position.dart';
import 'scrollable.dart';
import 'shortcuts.dart';
import 'single_child_scroll_view.dart';
import 'tap_region.dart';

// Examples can assume:
// late BuildContext context;
// late StateSetter setState;
// late List<Widget> menuItems;
// late RawMenuAnchorOverlayPosition position;

const bool _kDebugMenus = false;

const Map<ShortcutActivator, Intent> _kMenuTraversalShortcuts = <ShortcutActivator, Intent>{
  SingleActivator(LogicalKeyboardKey.gameButtonA): ActivateIntent(),
  SingleActivator(LogicalKeyboardKey.escape): DismissIntent(),
  SingleActivator(LogicalKeyboardKey.arrowDown): DirectionalFocusIntent(TraversalDirection.down),
  SingleActivator(LogicalKeyboardKey.arrowUp): DirectionalFocusIntent(TraversalDirection.up),
  SingleActivator(LogicalKeyboardKey.arrowLeft): DirectionalFocusIntent(TraversalDirection.left),
  SingleActivator(LogicalKeyboardKey.arrowRight): DirectionalFocusIntent(TraversalDirection.right),
  SingleActivator(LogicalKeyboardKey.home): _FocusFirstMenuItemIntent(),
  SingleActivator(LogicalKeyboardKey.end): _FocusLastMenuItemIntent(),
};

/// Anchor and menu positioning information passed to
/// [RawMenuAnchor.overlayBuilder].
@immutable
class RawMenuAnchorOverlayPosition {
  /// Creates a [RawMenuAnchorOverlayPosition].
  const RawMenuAnchorOverlayPosition({
    required this.anchorRect,
    required this.overlaySize,
    required this.tapRegionGroupId,
    this.position,
  });

  /// The global position of the anchor widget that the menu is attached to,
  /// relative to the [overlaySize].
  final ui.Rect anchorRect;

  /// The size of the overlay that the menu is being rendered in.
  final ui.Size overlaySize;

  /// The `position` argument passed to [MenuController.open].
  ///
  /// The position describes the distance from the top-left corner
  /// of the anchor that the menu should be positioned at.
  final Offset? position;

  /// The group ID of the tap region that should be used to consume taps outside
  /// of the menu.
  // This used to be a separate parameter, but was moved into the position class
  // to keep the constructor API cleaner.
  final Object tapRegionGroupId;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }

    if (other.runtimeType != runtimeType) {
      return false;
    }

    return other is RawMenuAnchorOverlayPosition &&
        other.anchorRect == anchorRect &&
        other.overlaySize == overlaySize &&
        other.position == position &&
        other.tapRegionGroupId == tapRegionGroupId;
  }

  @override
  int get hashCode {
    return Object.hash(anchorRect, overlaySize, position, tapRegionGroupId);
  }
}

/// The type of builder function used by [RawMenuAnchor.overlayBuilder] to build
/// the overlay attached to a [RawMenuAnchor].
///
/// The `context` is the context that the overlay is being built in.
///
/// The `menuChildren` is the list of children containing the menu items that
/// was passed to the [RawMenuAnchor].
///
/// The `position` describes the position of the menu overlay for the
/// [RawMenuAnchor.overlayBuilder] constructor.
typedef RawMenuAnchorOverlayBuilder =
    Widget Function(BuildContext context, RawMenuAnchorOverlayPosition position);

/// The type of builder function used by [RawMenuAnchor.builder] to build the
/// widget that the [RawMenuAnchor] surrounds.
///
/// The `context` is the context in which the anchor is being built.
///
/// The `controller` is the [MenuController] that can be used to open and close
/// the menu.
///
/// The `child` is an optional child supplied as the [RawMenuAnchor.child]
/// attribute. The child is intended to be incorporated in the result of the
/// function.
typedef RawMenuAnchorChildBuilder =
    Widget Function(BuildContext context, MenuController controller, Widget? child);

// An inherited widget that provides the [RawMenuAnchor] to its descendants.
//
// Used to notify anchor descendants when the menu opens and closes, and to
// access the anchor's controller.
class _RawMenuAnchorScope extends InheritedWidget {
  const _RawMenuAnchorScope({
    required this.anchor,
    required this.isOpen,
    required this.controller,
    required super.child,
  });

  final _RawMenuAnchorState anchor;
  final bool isOpen;
  final MenuController controller;

  @override
  bool updateShouldNotify(_RawMenuAnchorScope oldWidget) {
    return anchor != oldWidget.anchor ||
        isOpen != oldWidget.isOpen ||
        controller != oldWidget.controller;
  }
}

/// A widget used to mark the "anchor" for a set of submenus, defining the
/// rectangle used to position the menu, which can be done either with an
/// explicit location, or with an alignment.
///
/// The default [RawMenuAnchor] constructor creates a simple menu overlay that
/// positions and displays a `panel` widget when opened. Focus, gestures, and
/// semantics are handled internally, whereas the appearance and layout of the
/// menu are handled by the `panel`.
///
/// While any widget can be used as a `panel`, the [RawMenuPanel] widget
/// provides a basic vertical layout with a customizable appearance.
///
/// To completely customize the overlay, the [RawMenuAnchor.overlayBuilder]
/// delegates overlay construction to a builder function. This constructor is
/// useful for creating complex menu overlays with custom animations or layouts,
/// but requires managing positioning, semantics, and interactions. The builder
/// is called with a [RawMenuAnchorOverlayPosition] that defines the anchor's
/// rect, the size of the overlay, the [TapRegion.groupId] for the menu system,
/// and the [position] argument passed to [MenuController.open].
///
/// Finally, the [RawMenuAnchor.node] constructor can be used to create menus
/// that are always visible and are not displayed in an [OverlayPortal]. This is
/// useful for creating a menu bars that display submenus when hovered over.
///
/// {@tool snippet}
///
/// This example uses a [RawMenuAnchor] to create a simple edit menu.
///
/// ```dart
/// RawMenuAnchor(
///   padding: const EdgeInsets.symmetric(vertical: 4),
///   alignmentOffset: const Offset(0, 6),
///   builder: (
///     BuildContext context,
///     MenuController controller,
///     Widget? child,
///   ) {
///     return TextButton(
///       onPressed: () {
///         if (controller.isOpen) {
///           controller.close();
///         } else {
///           controller.open();
///         }
///       },
///       child: const Text('Edit'),
///     );
///   },
///   panel: RawMenuPanel(
///     padding: const EdgeInsets.symmetric(vertical: 4),
///     constraints: const BoxConstraints(minWidth: 200),
///     decoration: RawMenuPanel.lightSurfaceDecoration,
///     menuChildren: <Widget>[
///       MenuItemButton(onPressed: () {}, child: const Text('Undo')),
///       MenuItemButton(onPressed: () {}, child: const Text('Redo')),
///       const Divider(),
///       MenuItemButton(onPressed: () {}, child: const Text('Cut')),
///       MenuItemButton(onPressed: () {}, child: const Text('Copy')),
///       MenuItemButton(onPressed: () {}, child: const Text('Paste')),
///       MenuItemButton(onPressed: () {}, child: const Text('Delete')),
///       MenuItemButton(onPressed: () {}, child: const Text('Select All')),
///     ],
///   ),
/// );
/// ```
/// {@end-tool}
///
/// {@tool dartpad}
///
/// This example uses a [RawMenuAnchor] to build a simple menu with three items.
/// The "Edit" button opens and closes the menu when pressed. Selecting a menu
/// item will close the menu and update the selected item text.
///
/// ** See code in examples/api/lib/widgets/raw_menu_anchor/raw_menu_anchor.0.dart **
///
/// {@end-tool}
///
/// {@tool dartpad}
///
/// This example uses a [RawMenuAnchor] to build a context menu with a nested
/// submenu. Right-clicking the background opens and positions the context menu
/// at the cursor. Selecting a menu item will close the menu and update the
/// selected item text.
///
/// ** See code in examples/api/lib/widgets/raw_menu_anchor/raw_menu_anchor.1.dart **
///
/// {@end-tool}
///
class RawMenuAnchor extends StatelessWidget {
  /// Creates a [RawMenuAnchor].
  ///
  /// The `panel` argument is required.
  const RawMenuAnchor({
    super.key,
    this.controller,
    this.childFocusNode,
    this.alignment,
    this.alignmentOffset = Offset.zero,
    this.menuAlignment,
    this.consumeOutsideTaps = false,
    this.onOpen,
    this.onClose,
    bool this.useRootOverlay = false,
    String? semanticLabel,
    this.padding,
    this.builder,
    required Widget panel,
    this.child,
  }) : _semanticLabel = semanticLabel,
       _overlayBuilder = null,
       _panel = panel,
       _nodeBuilder = null;

  /// A [RawMenuAnchor] that delegates overlay construction to an
  /// `overlayBuilder`.
  ///
  /// The builder is called with a [RawMenuAnchorOverlayPosition] that defines
  /// the anchor's rect, the size of the overlay, the [TapRegion.groupId] for
  /// the menu system, and the position argument passed to
  /// [MenuController.open].
  ///
  /// To ensure taps within the menu register as within the menu system, the
  /// `tapRegionGroupId` should be passed to a [TapRegion] widget:
  ///
  /// ```dart
  /// TapRegion(
  ///   groupId: position.tapRegionGroupId,
  ///   onTapOutside: (PointerDownEvent event) {
  ///     MenuController.maybeOf(context)?.close();
  ///   },
  ///   child: Column(children: menuItems),
  /// )
  /// ```
  ///
  /// {@tool dartpad}
  /// This example uses a [RawMenuAnchor.overlayBuilder] to build an animated
  /// select menu with four items. The menu opens with the last selected item
  /// positioned above the anchor.
  ///
  /// ** See code in examples/api/lib/widgets/raw_menu_anchor/raw_menu_anchor.2.dart **
  /// {@end-tool}
  const RawMenuAnchor.overlayBuilder({
    super.key,
    this.controller,
    this.childFocusNode,
    this.consumeOutsideTaps = false,
    this.onOpen,
    this.onClose,
    bool this.useRootOverlay = false,
    required RawMenuAnchorOverlayBuilder overlayBuilder,
    this.builder,
    this.child,
  }) : alignment = null,
       menuAlignment = null,
       alignmentOffset = Offset.zero,
       padding = null,
       _panel = null,
       _overlayBuilder = overlayBuilder,
       _semanticLabel = null,
       _nodeBuilder = null;

  /// Creates a [RawMenuAnchor] whose anchor is a menu.
  ///
  /// Unlike [RawMenuAnchor.overlayBuilder], [RawMenuAnchor.node] does not have
  /// an overlay. As a result, calling [MenuController.open] does nothing, and
  /// calling [MenuController.close] will close all children of this anchor.
  /// [MenuController.isOpen] will only return true if a child of this anchor is
  /// open.
  ///
  /// Because [RawMenuAnchor.node] does not provide semantics or focus
  /// management, the [MenuBar] widget is the recommended way of creating a menu
  /// bar. However, [RawMenuAnchor.node] can be useful in cases where finer
  /// control over focus behavior is needed, or where a custom layout (such as a
  /// vertical menu bar) is desired,
  ///
  /// The [builder] argument is required.
  ///
  /// {@tool snippet}
  ///
  /// This snippet renders a horizontal [RawMenuAnchor.node] with 5 fly-out
  /// submenus.
  ///
  /// ```dart
  /// RawMenuAnchor.node(
  ///   child: Row(
  ///     mainAxisSize: MainAxisSize.min,
  ///     children: <Widget>[
  ///       for (int i = 0; i < 5; i++)
  ///         Builder(
  ///           builder: (BuildContext context) {
  ///             final MenuController? nodeController = MenuController.maybeOf(context);
  ///             return RawMenuAnchor(
  ///               builder: (
  ///                 BuildContext context,
  ///                 MenuController controller,
  ///                 Widget? child,
  ///               ) {
  ///                 return MenuItemButton(
  ///                   onHover: (bool value) {
  ///                     // Only open the submenu if the menubar is open.
  ///                     if (nodeController?.isOpen ?? false) {
  ///                       controller.open();
  ///                     }
  ///                   },
  ///                   onPressed: () {
  ///                     if (controller.isOpen) {
  ///                       controller.close();
  ///                     } else {
  ///                       controller.open();
  ///                     }
  ///                   },
  ///                   child: Text('Submenu $i  ${controller.isOpen ? '▲' : '▼'}'),
  ///                 );
  ///               },
  ///               panel: RawMenuPanel(
  ///                 decoration: RawMenuPanel.lightSurfaceDecoration,
  ///                 menuChildren: <Widget>[
  ///                   for (int j = 0; j < 5; j++)
  ///                     MenuItemButton(
  ///                       onPressed: nodeController?.close,
  ///                       child: Align(
  ///                         alignment: Alignment.centerLeft,
  ///                         child: Text('Menu Item $i.$j'),
  ///                       ),
  ///                     ),
  ///                 ],
  ///               ),
  ///             );
  ///           },
  ///         )
  ///     ],
  ///   ),
  ///   builder: (BuildContext context, Widget? child) {
  ///     return Container(
  ///       decoration: BoxDecoration(border: Border.all()),
  ///       child: child,
  ///     );
  ///   },
  /// );
  /// ```
  /// {@end-tool}
  ///
  /// {@tool dartpad}
  ///
  /// This example uses [RawMenuAnchor.node] to build a menu bar with four
  /// submenus. Hovering over menu items opens their respective submenus.
  /// Selecting a menu item will close the menu and update the selected item
  /// text.
  ///
  /// ** See code in examples/api/lib/widgets/raw_menu_anchor/raw_menu_anchor.3.dart **
  /// {@end-tool}
  const RawMenuAnchor.node({super.key, this.controller, this.child, TransitionBuilder? builder})
    : assert(builder != null || child != null, 'Either a builder or a child must be provided.'),
      _overlayBuilder = null,
      alignment = null,
      menuAlignment = null,
      alignmentOffset = Offset.zero,
      onOpen = null,
      onClose = null,
      childFocusNode = null,
      consumeOutsideTaps = false,
      _semanticLabel = null,
      useRootOverlay = null,
      _panel = null,
      padding = null,
      builder = null,
      _nodeBuilder = builder;

  /// An optional [MenuController] that allows opening and closing of the menu
  /// from other widgets.
  ///
  /// If not supplied, a new [MenuController] will be created and managed by the
  /// [RawMenuAnchor].
  final MenuController? controller;

  /// The [FocusNode] attached to the widget that takes focus when the
  /// menu is opened or closed.
  ///
  /// If not supplied, the anchor will not be focused when the menu is opened.
  final FocusNode? childFocusNode;

  /// Whether or not a tap event that closes the menu will be permitted to
  /// continue on to the gesture arena.
  ///
  /// If false, then tapping outside of a menu when the menu is open will both
  /// close the menu, and allow the tap to participate in the gesture arena.
  ///
  /// If true, then it will only close the menu, and the tap event will be
  /// consumed.
  ///
  /// Defaults to false.
  final bool consumeOutsideTaps;

  /// A callback that is invoked when the menu is opened.
  final VoidCallback? onOpen;

  /// A callback that is invoked when the menu is closed.
  final VoidCallback? onClose;

  /// The widget that this [RawMenuAnchor] surrounds.
  ///
  /// Typically, this is a button used to open the menu by calling
  /// [MenuController.open] on the `controller` passed to the builder.
  ///
  /// If not supplied, then the [RawMenuAnchor] will be the size that its parent
  /// allocates for it.
  final RawMenuAnchorChildBuilder? builder;

  /// The optional child to be passed to the [builder].
  ///
  /// Supply this child if there is a portion of the widget tree built in
  /// [builder] that doesn't depend on the `controller` or `context` supplied to
  /// the [builder]. It will be more efficient, since Flutter doesn't then need
  /// to rebuild this child when those change.
  final Widget? child;

  /// The point on the anchor surface that attaches to the menu.
  ///
  /// The [alignment] is ignored if a `position` argument is provided to
  /// [MenuController.open].
  ///
  /// If the menu overflows the edge of the screen, the menu will be flipped
  /// across the anchor's midpoint on the axis of overflow, effectively negating
  /// the alignment on that axis. For example, if the menu on the right side of
  /// the anchor overflows the right edge of the screen, the menu will be
  /// flipped to the left side of the anchor.
  ///
  /// Defaults to [AlignmentDirectional.bottomStart].
  final AlignmentGeometry? alignment;

  /// The offset applied to the menu relative to the anchor attachment point.
  ///
  /// By default, increasing the [Offset.dx] and [Offset.dy] value of
  /// [alignmentOffset] will shift the menu position rightward and downward,
  /// respectively.
  ///
  /// However, when the [alignment] is an [AlignmentDirectional], increasing the
  /// [Offset.dx] value of [alignmentOffset] will shift the menu in the reading
  /// direction of the ambient [Directionality] -- rightward in
  /// [TextDirection.ltr] and leftward in [TextDirection.rtl].
  ///
  /// The [alignment] and [alignmentOffset] are ignored if a `position` argument
  /// is provided to [MenuController.open].
  ///
  /// Defaults to [Offset.zero].
  final Offset alignmentOffset;

  /// The [EdgeInsetsGeometry] applied to the menu surface but ignored during
  /// menu positioning.
  ///
  /// Menus commonly apply padding to the top and bottom of the menu surface,
  /// which can cause a submenu's items to be vertically misaligned with their
  /// parent menu items. To ensure a submenu's items align with their parent's
  /// items, the [padding] applied to the menu surface is ignored when
  /// calculating the position of the menu.
  ///
  /// Defaults to null.
  final EdgeInsetsGeometry? padding;

  /// The point on the menu surface that attaches to the anchor.
  ///
  /// Unlike [alignment] and [alignmentOffset], the [menuAlignment] will be
  /// applied when the menu is opened with a `position` argument.
  ///
  /// Defaults to [AlignmentDirectional.topStart].
  final AlignmentGeometry? menuAlignment;

  // The semanticLabel argument is used by accessibility frameworks to announce
  // the name of the menu.
  final String? _semanticLabel;

  /// {@template flutter.widgets.RawMenuAnchor.useRootOverlay}
  /// Whether the menu panel should be rendered in the root [Overlay].
  ///
  /// When true, the menu is mounted in the root overlay. Rendering the menu in
  /// the root overlay prevents the menu from being obscured by other widgets.
  ///
  /// When false, the menu is rendered in the nearest ancestor [Overlay].
  ///
  /// Submenus will always use the same overlay as their top-level ancestor, so
  /// setting a [useRootOverlay] value on a submenu will have no effect.
  /// {@endtemplate}
  ///
  /// Defaults to false on overlay menus.
  final bool? useRootOverlay;

  // The menu panel that is displayed when the menu is opened.
  //
  // The panel should lay out its menu children in a vertical list.
  final Widget? _panel;
  final TransitionBuilder? _nodeBuilder;
  final RawMenuAnchorOverlayBuilder? _overlayBuilder;

  /// The overlay builder used by the default [RawMenuAnchor] constructor.
  ///
  /// The [defaultOverlayBuilder] constructor builds a simple menu overlay. An
  /// internal [FocusScope] is created to manage focus. Default keyboard
  /// shortcuts include:
  ///
  /// - `ArrowUp` moves focus to the previous menu item.
  /// - `ArrowDown` moves focus to the next menu item.
  /// - `ArrowLeft` moves focus out of a submenu, thereby closing that submenu.
  /// - `ArrowRight` opens and moves focus into a submenu.
  /// - `Home` moves focus to the first menu item.
  /// - `End` moves focus to the last menu item.
  /// - `Escape` closes all menu layers.
  ///
  /// To customize the overlay, use the [RawMenuAnchor.overlayBuilder]
  /// constructor.
  Widget defaultOverlayBuilder(BuildContext context, RawMenuAnchorOverlayPosition position) {
    return _MenuOverlay(
      position: position,
      alignmentOffset: alignmentOffset,
      alignment: alignment,
      menuAlignment: menuAlignment,
      consumeOutsideTaps: consumeOutsideTaps,
      padding: padding,
      semanticLabel: _semanticLabel,
      child: _panel!,
    );
  }

  @override
  Widget build(BuildContext context) {
    if (useRootOverlay == null) {
      return _RawMenuAnchorNode(
        controller: controller,
        consumeOutsideTaps: consumeOutsideTaps,
        builder: _nodeBuilder,
        child: child,
      );
    }

    return _RawMenuAnchorOverlay(
      useRootOverlay: useRootOverlay!,
      controller: controller,
      childFocusNode: childFocusNode,
      consumeOutsideTaps: consumeOutsideTaps,
      onOpen: onOpen,
      onClose: onClose,
      padding: padding,
      overlayBuilder: _overlayBuilder ?? defaultOverlayBuilder,
      // If there's a custom overlay, then that overlay will manage its own
      // focus scope.
      hasExternalFocusScope: _overlayBuilder != null,
      builder: builder,
      child: child,
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(ObjectFlagProperty<MenuController>.has('controller', controller));
    if (useRootOverlay != null) {
      properties.add(ObjectFlagProperty<FocusNode>.has('focusNode', childFocusNode));
      if (_overlayBuilder == null) {
        properties.add(
          DiagnosticsProperty<EdgeInsetsGeometry>('padding', padding, defaultValue: null),
        );
        properties.add(
          DiagnosticsProperty<Offset>(
            'alignmentOffset',
            alignmentOffset,
            defaultValue: Offset.zero,
          ),
        );
      }
    }
  }
}

/// A simple menu surface that displays a vertical list of menu items.
///
/// The [RawMenuPanel] is painted with a dark theme when
/// [MediaQuery.maybePlatformBrightnessOf] returns [Brightness.dark], and a
/// light theme when the brightness is [Brightness.light] or null. To override
/// this behavior, a [decoration] can be provided.
///
/// Any [padding] applied to the [RawMenuAnchor] is inherited by [RawMenuPanel].
/// This behavior can be overridden by supplying a custom [padding].
///
/// The [RawMenuPanel] is only responsible for the size, appearance, and layout
/// of menu items. To manage the positioning, semantics, and interaction of the
/// menu overlay, the [RawMenuAnchor.overlayBuilder] constructor should be used.
///
/// See also:
///
///  * [RawMenuAnchor], for a widget that creates a menu anchor that can be
///    paired with a [RawMenuPanel].
///  * [RawMenuAnchor.overlayBuilder], for a widget that creates a menu anchor
///    with a custom overlay.
/// * [RawMenuAnchor.node], for a widget that creates a menu that is always
///    visible and is not displayed in an [OverlayPortal].
/// * [MenuAnchor], a material design menu anchor.
class RawMenuPanel extends StatelessWidget {
  /// Creates a [RawMenuPanel].
  ///
  /// The [menuChildren] argument is required.
  const RawMenuPanel({
    super.key,
    this.clipBehavior = Clip.antiAlias,
    this.constraints,
    this.constrainCrossAxis = false,
    this.decoration,
    this.padding,
    required this.menuChildren,
  });

  /// The [Decoration] used to paint the menu surface.
  ///
  /// Defaults to [lightSurfaceDecoration] when
  /// [MediaQuery.platformBrightnessOf] returns [Brightness.light] or null, and
  /// [darkSurfaceDecoration] when [MediaQuery.platformBrightnessOf]
  /// returns [Brightness.dark].
  final Decoration? decoration;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.hardEdge].
  final Clip clipBehavior;

  /// The constraints to apply to the menu surface.
  ///
  /// If null, the menu will be allowed to expand to the intrinsic size of its
  /// children.
  final BoxConstraints? constraints;

  /// The menu items that should be displayed by this [RawMenuPanel].
  final List<Widget> menuChildren;

  /// Whether the menu's cross axis should be laid out with regard to the bounds
  /// of the overlay.
  ///
  /// When true, the width of the menu will be constrained by the width of the
  /// overlay. This can cause the menu contents to wrap.
  ///
  /// When false, the menu will be allowed to expand to the intrinsic size of
  /// its children, and menu items that overflow will be visually clipped.
  ///
  /// Defaults to false.
  final bool constrainCrossAxis;

  /// The [EdgeInsetsGeometry] applied to the menu surface.
  ///
  /// When a [RawMenuPanel] is used with a [RawMenuAnchor], [padding] applied to
  /// the menu surface can be ignored during layout by supplying an equivalent
  /// amount of [padding] to the [RawMenuAnchor] constructor. This is useful
  /// when aligning a submenu with its anchor.
  ///
  /// Defaults to null, which applies no padding.
  final EdgeInsetsGeometry? padding;

  /// The surface decoration painted by the [RawMenuPanel] constructor when
  /// [MediaQuery.maybePlatformBrightnessOf] returns null or [Brightness.light].
  static const Decoration lightSurfaceDecoration = BoxDecoration(
    borderRadius: BorderRadius.all(Radius.circular(6.0)),
    color: ui.Color.fromARGB(255, 253, 253, 253),
    border: Border.fromBorderSide(
      BorderSide(color: ui.Color.fromARGB(255, 255, 255, 255), width: 0.5),
    ),
    boxShadow: <BoxShadow>[
      BoxShadow(color: ui.Color.fromARGB(30, 0, 0, 0), offset: Offset(0, 2), blurRadius: 6.0),
      BoxShadow(
        color: ui.Color.fromARGB(12, 0, 0, 0),
        offset: Offset(0, 6),
        spreadRadius: 8,
        blurRadius: 12.0,
      ),
    ],
  );

  /// The surface decoration painted by the [RawMenuPanel] when
  /// [MediaQuery.maybePlatformBrightnessOf] returns [Brightness.dark].
  static const Decoration darkSurfaceDecoration = BoxDecoration(
    borderRadius: BorderRadius.all(Radius.circular(6.0)),
    color: ui.Color.fromARGB(255, 32, 33, 36),
    border: Border.fromBorderSide(BorderSide(color: ui.Color.fromARGB(200, 0, 0, 0), width: 0.5)),
    boxShadow: <BoxShadow>[
      BoxShadow(color: ui.Color.fromARGB(45, 0, 0, 0), offset: Offset(0, 1), blurRadius: 4.0),
      BoxShadow(color: ui.Color.fromARGB(65, 0, 0, 0), offset: Offset(0, 4), blurRadius: 12.0),
    ],
  );

  Decoration _resolveDecoration(BuildContext context) {
    return decoration ??
        switch (MediaQuery.maybePlatformBrightnessOf(context)) {
          Brightness.dark => darkSurfaceDecoration,
          Brightness.light || null => lightSurfaceDecoration,
        };
  }

  EdgeInsetsGeometry? _resolvePadding(BuildContext context) {
    return padding ??
        switch (MenuController.maybeOf(context)?._anchor?.widget) {
          final _RawMenuAnchor anchor => anchor.padding,
          _ => null,
        };
  }

  @override
  Widget build(BuildContext context) {
    final Widget body = SingleChildScrollView(
      clipBehavior: Clip.none,
      child: ListBody(children: menuChildren),
    );

    Widget child = IntrinsicWidth(
      child: Builder(
        builder: (BuildContext context) {
          return Container(
            clipBehavior: clipBehavior,
            padding: _resolvePadding(context),
            decoration: _resolveDecoration(context),
            child: body,
          );
        },
      ),
    );

    if (constraints != null) {
      child = ConstrainedBox(constraints: constraints!, child: child);
    }

    // The menu's items will be constrained to the size of the overlay,
    // causing them to wrap if they overflow.
    if (constrainCrossAxis) {
      return child;
    }

    // The menu's items can grow beyond the size of the overlay, but will be
    // clipped by the overlay's bounds.
    return UnconstrainedBox(
      clipBehavior: Clip.hardEdge,
      alignment: AlignmentDirectional.centerStart,
      constrainedAxis: Axis.vertical,
      child: child,
    );
  }

  @override
  List<DiagnosticsNode> debugDescribeChildren() {
    return <DiagnosticsNode>[for (final Widget child in menuChildren) child.toDiagnosticsNode()];
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties
      ..add(DiagnosticsProperty<Clip>('clipBehavior', clipBehavior))
      ..add(DiagnosticsProperty<bool>('constrainCrossAxis', constrainCrossAxis));
    if (padding != null) {
      properties.add(DiagnosticsProperty<EdgeInsetsGeometry>('padding override', padding));
    }
  }
}

// Base class that provides the common interface and state for the different
// types of RawMenuAnchors, [_RawMenuAnchorOverlay] and [_RawMenuAnchorNode].
sealed class _RawMenuAnchor extends StatefulWidget {
  const _RawMenuAnchor({
    this.childFocusNode,
    required this.consumeOutsideTaps,
    this.controller,
    this.padding,
  });
  final FocusNode? childFocusNode;
  final bool consumeOutsideTaps;
  final MenuController? controller;
  final EdgeInsetsGeometry? padding;

  @override
  State<_RawMenuAnchor> createState();
}

@optionalTypeArgs
sealed class _RawMenuAnchorState<T extends _RawMenuAnchor> extends State<T> {
  final List<_RawMenuAnchorState> _anchorChildren = <_RawMenuAnchorState>[];
  _RawMenuAnchorState? _parent;
  ScrollPosition? _scrollPosition;
  Size? _viewSize;
  MenuController? _internalMenuController;
  MenuController get _menuController {
    return widget.controller ?? (_internalMenuController ??= MenuController());
  }

  bool get _isRoot => _parent == null;
  bool get _isOpen;
  _RawMenuAnchorState get _root {
    _RawMenuAnchorState anchor = this;
    while (anchor._parent != null) {
      anchor = anchor._parent!;
    }
    return anchor;
  }

  @override
  void initState() {
    super.initState();
    _menuController._attach(this);
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    final _RawMenuAnchorState? newParent = MenuController.maybeOf(context)?._anchor;
    if (newParent != _parent) {
      assert(
        newParent != this,
        'A MenuController should only be attached to one anchor at a time.',
      );
      _parent?._removeChild(this);
      _parent = newParent;
      _parent?._addChild(this);
    }

    _scrollPosition?.isScrollingNotifier.removeListener(_handleScroll);
    _scrollPosition = Scrollable.maybeOf(context)?.position;
    _scrollPosition?.isScrollingNotifier.addListener(_handleScroll);
    final Size newSize = MediaQuery.sizeOf(context);
    if (_viewSize != null && newSize != _viewSize) {
      // Close the menus if the view changes size.
      _root._close();
    }
    _viewSize = newSize;
  }

  @override
  void didUpdateWidget(T oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.controller != widget.controller) {
      (oldWidget.controller ?? _internalMenuController)?._detach(this);
      _menuController._attach(this);
      if (widget.controller != null) {
        _internalMenuController = null;
      }
    }
    assert(_menuController._anchor == this);
    assert(widget.controller == null || _internalMenuController == null);
  }

  @override
  void dispose() {
    assert(_debugMenuInfo('Disposing of $this'));
    if (_isOpen) {
      _close(inDispose: true);
    }

    _parent?._removeChild(this);
    _parent = null;
    _anchorChildren.clear();
    _menuController._detach(this);
    _internalMenuController = null;
    super.dispose();
  }

  void _addChild(_RawMenuAnchorState child) {
    assert(_isRoot || _debugMenuInfo('Added root child: $child'));
    assert(!_anchorChildren.contains(child));
    _anchorChildren.add(child);
    assert(_debugMenuInfo('Added:\n${child.widget.toStringDeep()}'));
    assert(_debugMenuInfo('Tree:\n${widget.toStringDeep()}'));
  }

  void _removeChild(_RawMenuAnchorState child) {
    assert(_isRoot || _debugMenuInfo('Removed root child: $child'));
    assert(_anchorChildren.contains(child));
    assert(_debugMenuInfo('Removing:\n${child.widget.toStringDeep()}'));
    _anchorChildren.remove(child);
    assert(_debugMenuInfo('Tree:\n${widget.toStringDeep()}'));
  }

  void _handleScroll() {
    // If an ancestor scrolls, and we're a root anchor, then close the menus.
    // Don't just close it on *any* scroll, since we want to be able to scroll
    // menus themselves if they're too big for the view.
    if (_isRoot) {
      _close();
    }
  }

  void _childChangedOpenState() {
    _parent?._childChangedOpenState();
    if (SchedulerBinding.instance.schedulerPhase != SchedulerPhase.persistentCallbacks) {
      setState(() {
        // Mark dirty now, but only if not in a build.
      });
    } else {
      SchedulerBinding.instance.addPostFrameCallback((Duration timestamp) {
        setState(() {
          // Mark dirty
        });
      });
    }
  }

  /// Open the menu, optionally at a position relative to the [RawMenuAnchor].
  ///
  /// Call this when the menu should be shown to the user.
  ///
  /// The optional `position` argument will specify the location of the menu in
  /// the local coordinates of the [RawMenuAnchor], ignoring any
  /// [MenuStyle.alignment] and/or [RawMenuAnchor.alignmentOffset] that were
  /// specified.
  void _open({Offset? position});
  void _close({bool inDispose = false});
  void _closeChildren({bool inDispose = false}) {
    assert(_debugMenuInfo('Closing children of $this${inDispose ? ' (dispose)' : ''}'));
    for (final _RawMenuAnchorState child in List<_RawMenuAnchorState>.from(_anchorChildren)) {
      child._close(inDispose: inDispose);
    }
  }

  void _handleOutsideTap(PointerDownEvent pointerDownEvent) {
    assert(_debugMenuInfo('Tapped Outside ${widget.controller}'));
    _closeChildren();
  }

  // Used to build the anchor widget in subclasses.
  Widget _buildAnchor(BuildContext context);

  @override
  @nonVirtual
  Widget build(BuildContext context) {
    return _RawMenuAnchorScope(
      anchor: this,
      isOpen: _isOpen,
      controller: _menuController,
      child: Actions(
        actions: <Type, Action<Intent>>{
          // Check if open to allow DismissIntent to bubble when the menu is
          // closed.
          if (_isOpen) DismissIntent: DismissMenuAction(controller: _menuController),
        },
        child: Builder(builder: _buildAnchor),
      ),
    );
  }

  @override
  String toString({DiagnosticLevel? minLevel}) => describeIdentity(this);
}

class _RawMenuAnchorOverlay extends _RawMenuAnchor {
  const _RawMenuAnchorOverlay({
    this.onOpen,
    this.onClose,
    this.hasExternalFocusScope = false,
    this.useRootOverlay = false,
    required this.overlayBuilder,
    this.builder,
    this.child,
    super.padding,
    super.controller,
    super.childFocusNode,
    super.consumeOutsideTaps = false,
  });

  final VoidCallback? onOpen;
  final VoidCallback? onClose;
  final RawMenuAnchorChildBuilder? builder;
  final Widget? child;
  final RawMenuAnchorOverlayBuilder overlayBuilder;
  final bool useRootOverlay;

  // Whether focus is handled by this class (default overlay) or externally
  // (overlayBuilder).
  final bool hasExternalFocusScope;

  @override
  State<_RawMenuAnchorOverlay> createState() => _RawMenuAnchorOverlayState();
}

class _RawMenuAnchorOverlayState extends _RawMenuAnchorState<_RawMenuAnchorOverlay> {
  static final Map<Type, Action<Intent>> _rootOverlayAnchorActions = <Type, Action<Intent>>{
    DirectionalFocusIntent: _AnchorDirectionalFocusAction(),
  };

  // This is the global key that is used later to determine the bounding rect
  // for the anchor's region that the CustomSingleChildLayout's delegate
  // uses to determine where to place the menu on the screen and to avoid the
  // view's edges.
  final GlobalKey _anchorKey = GlobalKey<_RawMenuAnchorOverlayState>(
    debugLabel: kReleaseMode ? null : 'MenuAnchor',
  );
  final OverlayPortalController _overlayController = OverlayPortalController(
    debugLabel: kReleaseMode ? null : 'MenuAnchor controller',
  );

  bool get useRootOverlay {
    if (_parent case _RawMenuAnchorOverlayState(useRootOverlay: final bool useRoot)) {
      return useRoot;
    }

    assert(_isRootAnchor);
    return widget.useRootOverlay;
  }

  Offset? _menuPosition;
  FocusNode? _menuFocusNode;
  FocusScopeNode? _menuScopeNode;
  bool get _isRootAnchor => _parent is! _RawMenuAnchorOverlayState;
  FocusTraversalPolicy? get _overlayTraversalPolicy {
    if (_menuScopeNode?.context?.mounted != true) {
      return null;
    }

    return FocusTraversalGroup.maybeOf(_menuScopeNode!.context!) ?? ReadingOrderTraversalPolicy();
  }

  FocusNode? get _firstFocus {
    assert(_menuScopeNode != null, '_firstFocus requires a menu scope node.');
    return _overlayTraversalPolicy?.findFirstFocus(_menuScopeNode!, ignoreCurrentFocus: true);
  }

  FocusNode? get _lastFocus {
    assert(_menuScopeNode != null, '_lastFocus requires a menu scope node.');
    return _overlayTraversalPolicy?.findLastFocus(_menuScopeNode!, ignoreCurrentFocus: true);
  }

  @override
  bool get _isOpen => _overlayController.isShowing;

  @override
  void initState() {
    super.initState();
    // If the overlay is custom, then focus is handled externally.
    if (!widget.hasExternalFocusScope) {
      _menuScopeNode = FocusScopeNode(
        debugLabel: kReleaseMode ? null : '${describeIdentity(this)} Sub Menu',
      );
      _menuFocusNode = FocusNode(
        debugLabel: kReleaseMode ? null : '${describeIdentity(this)} Focus Node',
      );
    }
  }

  @override
  void dispose() {
    _menuScopeNode?.dispose();
    _menuFocusNode?.dispose();
    super.dispose();
  }

  @override
  Widget _buildAnchor(BuildContext context) {
    Widget child = Shortcuts(
      includeSemantics: false,
      shortcuts: _kMenuTraversalShortcuts,
      child: TapRegion(
        groupId: _root._menuController,
        consumeOutsideTaps: _root._isOpen && widget.consumeOutsideTaps,
        onTapOutside: _handleOutsideTap,
        child: Builder(
          key: _anchorKey,
          builder: (BuildContext context) {
            return widget.builder?.call(context, _menuController, widget.child) ??
                widget.child ??
                const SizedBox();
          },
        ),
      ),
    );

    if (!widget.hasExternalFocusScope && _isRootAnchor) {
      child = Actions(
        actions: _isOpen ? _rootOverlayAnchorActions : <Type, Action<Intent>>{},
        child: child,
      );
    }

    child =
        useRootOverlay
            ? OverlayPortal.targetsRootOverlay(
              controller: _overlayController,
              overlayChildBuilder: _buildOverlay,
              child: child,
            )
            : OverlayPortal(
              controller: _overlayController,
              overlayChildBuilder: _buildOverlay,
              child: child,
            );

    if (widget.hasExternalFocusScope) {
      return child;
    }

    // Focus is only used to monitor focus changes, so it's not necessary to
    // include semantics or allow focus to be requested.
    return Focus(
      focusNode: _menuFocusNode,
      includeSemantics: false,
      canRequestFocus: false,
      onFocusChange: _handleFocusChange,
      child: child,
    );
  }

  Widget _buildOverlay(BuildContext context) {
    final BuildContext anchorContext = _anchorKey.currentContext!;
    final RenderBox overlay =
        Overlay.of(anchorContext, rootOverlay: useRootOverlay).context.findRenderObject()!
            as RenderBox;
    final RenderBox anchorBox = anchorContext.findRenderObject()! as RenderBox;
    final ui.Offset upperLeft = anchorBox.localToGlobal(Offset.zero, ancestor: overlay);
    final ui.Offset bottomRight = anchorBox.localToGlobal(
      anchorBox.size.bottomRight(Offset.zero),
      ancestor: overlay,
    );
    return widget.overlayBuilder(
      context,
      RawMenuAnchorOverlayPosition(
        anchorRect: Rect.fromPoints(upperLeft, bottomRight),
        overlaySize: overlay.size,
        position: _menuPosition,
        tapRegionGroupId: _root._menuController,
      ),
    );
  }

  void _focusButton() {
    assert(_debugMenuInfo('Requesting focus for ${widget.childFocusNode}'));
    widget.childFocusNode?.requestFocus();
  }

  // Open the menu, optionally at a position relative to the [RawMenuAnchor].
  //
  // Call this when the menu should be shown to the user.
  //
  // The optional `position` argument will specify the location of the menu in
  // the local coordinates of the [RawMenuAnchor], ignoring any
  // [MenuStyle.alignment] and/or [RawMenuAnchor.alignmentOffset] that were
  // specified.
  @override
  void _open({Offset? position}) {
    assert(_menuController._anchor == this);
    if (_isOpen) {
      if (position == _menuPosition) {
        assert(_debugMenuInfo("Not opening $this because it's already open"));
        // The menu is open and not being moved, so just return.
        return;
      }

      // The menu is already open, but we need to move to another location, so
      // close it first.
      _close();
    }

    assert(_debugMenuInfo('Opening $this at ${position ?? Offset.zero}'));

    // Close all siblings.
    _parent?._closeChildren();
    assert(!_overlayController.isShowing);

    _parent?._childChangedOpenState();
    _menuPosition = position;
    _overlayController.show();

    if (_isRootAnchor) {
      _focusButton();
    }

    widget.onOpen?.call();
    if (mounted && SchedulerBinding.instance.schedulerPhase != SchedulerPhase.persistentCallbacks) {
      setState(() {
        // Mark dirty to notify MenuController dependents.
      });
    }
  }

  // Close the menu.
  //
  // Call this when the menu should be closed. Has no effect if the menu is
  // already closed.
  @override
  void _close({bool inDispose = false}) {
    assert(_debugMenuInfo('Closing $this'));
    if (!_isOpen) {
      return;
    }

    _closeChildren(inDispose: inDispose);
    // Don't hide if we're in the middle of a build.
    if (SchedulerBinding.instance.schedulerPhase != SchedulerPhase.persistentCallbacks) {
      _overlayController.hide();
    } else if (!inDispose) {
      SchedulerBinding.instance.addPostFrameCallback((_) {
        _overlayController.hide();
      }, debugLabel: 'MenuAnchor.hide');
    }

    if (!inDispose) {
      // Notify that _childIsOpen changed state, but only if not
      // currently disposing.
      _parent?._childChangedOpenState();
      widget.onClose?.call();
      if (mounted &&
          SchedulerBinding.instance.schedulerPhase != SchedulerPhase.persistentCallbacks) {
        setState(() {
          // Mark dirty, but only if mounted and not in a build.
        });
      }
    }
  }

  // Closes the menu if the focus changes to something outside of the menu.
  //
  // Only used by the default menu overlay.
  void _handleFocusChange(bool value) {
    if (!_menuFocusNode!.hasFocus && !_menuScopeNode!.hasFocus) {
      _close();
    }
  }

  @override
  String toString({DiagnosticLevel? minLevel}) {
    return describeIdentity(this);
  }
}

class _RawMenuAnchorNode extends _RawMenuAnchor {
  const _RawMenuAnchorNode({
    super.consumeOutsideTaps = false,
    super.controller,
    this.child,
    required this.builder,
  });

  final Widget? child;
  final TransitionBuilder? builder;

  @override
  State<_RawMenuAnchorNode> createState() => _RawMenuAnchorNodeState();
}

class _RawMenuAnchorNodeState extends _RawMenuAnchorState<_RawMenuAnchorNode> {
  @override
  bool get _isOpen => _anchorChildren.any((_RawMenuAnchorState child) => child._isOpen);

  @override
  void _close({bool inDispose = false}) {
    if (!_isOpen) {
      return;
    }
    _closeChildren(inDispose: inDispose);
    if (!inDispose) {
      if (SchedulerBinding.instance.schedulerPhase != SchedulerPhase.persistentCallbacks) {
        setState(() {
          // Mark dirty, but only if mounted and not in a build.
        });
      } else {
        SchedulerBinding.instance.addPostFrameCallback((Duration timestamp) {
          if (mounted) {
            setState(() {
              // Mark dirty.
            });
          }
        });
      }
    }
  }

  @override
  void _open({Offset? position}) {
    assert(_menuController._anchor == this);
    // Menu nodes are always open, so this is a no-op.
    return;
  }

  @override
  Widget _buildAnchor(BuildContext context) {
    return TapRegion(
      groupId: _root._menuController,
      consumeOutsideTaps: _root._isOpen && widget.consumeOutsideTaps,
      onTapOutside: _handleOutsideTap,
      child: widget.builder?.call(context, widget.child) ?? widget.child,
    );
  }
}

/// A controller used to manage a menu created by a [MenuBar], [MenuAnchor], or
/// a [RawMenuAnchor].
///
/// A [MenuController] is used to control and interrogate a menu after it has
/// been created, with methods such as [open] and [close], and state accessors
/// like [isOpen].
///
/// [MenuController.maybeOf] can be used to retrieve a controller from the
/// [BuildContext] of a widget that is a descendant of a [MenuAnchor],
/// [MenuBar], [SubmenuButton], or [RawMenuAnchor]. By doing so, the widget will
/// establish a dependency relationship that will rebuild the widget when the
/// parent menu opens and closes.
///
/// {@tool snippet}
///
/// This example demonstrates how to use a [MenuController.maybeOf] to open and
/// close a menu from a descendent [BuildContext] of a [RawMenuAnchor].
///
/// ```dart
/// RawMenuAnchor(
///   panel: RawMenuPanel(
///     menuChildren: <Widget>[
///       Builder(builder: (BuildContext context) {
///         return MenuItemButton(
///           onPressed: () {
///             MenuController.maybeOf(context)?.close();
///           },
///           child: const Text('Close'),
///         );
///       })
///     ],
///   ),
///   child: Builder(builder: (BuildContext context) {
///     final MenuController controller = MenuController.maybeOf(context)!;
///     return TextButton(
///       onPressed: () {
///         if (controller.isOpen) {
///           controller.close();
///         } else {
///           controller.open();
///         }
///       },
///       child: const Text('Menu'),
///     );
///   }),
/// );
/// ```
/// {@end-tool}
///
/// See also:
///
/// * [MenuAnchor], a RawMenuAnchor that follows the Material Design guidelines.
/// * [MenuBar], a widget that creates a menu bar, that can take an optional
///   [MenuController].
/// * [SubmenuButton], a widget that has a button that manages a submenu.
/// * [RawMenuAnchor], a widget that defines a region that has submenu.
class MenuController {
  /// The anchor that this controller controls.
  ///
  /// This is set automatically when a [MenuController] is given to the anchor
  /// it controls.
  _RawMenuAnchorState? _anchor;

  /// Whether or not the menu associated with this [MenuController] is open.
  bool get isOpen {
    return _anchor?._isOpen ?? false;
  }

  /// Opens the menu that this [MenuController] is associated with.
  ///
  /// If `position` is given, then the menu will open at the position given, in
  /// the coordinate space of the root overlay.
  ///
  /// If given, the `position` will override the [RawMenuAnchor.alignmentOffset]
  /// given to the [RawMenuAnchor].
  ///
  /// If the menu's anchor point (either a [MenuBar], [MenuAnchor], or a
  /// [RawMenuAnchor]) is scrolled by an ancestor, or the view changes size,
  /// then any open menu will automatically close.
  void open({Offset? position}) {
    assert(_anchor != null);
    _anchor!._open(position: position);
  }

  /// Close the menu that this [MenuController] is associated with.
  ///
  /// Associating with a menu is done by passing a [MenuController] to a
  /// [RawMenuAnchor]. A [MenuController] is also be received by the
  /// [RawMenuAnchor.builder] when invoked.
  ///
  /// If the menu's anchor point (either a [MenuBar], [MenuAnchor], or a
  /// [RawMenuAnchor]) is scrolled by an ancestor, or the view changes size,
  /// then any open menu will automatically close.
  void close() {
    _anchor?._close();
  }

  /// Close the children of the menu associated with this [MenuController],
  /// without closing the menu itself.
  void closeChildren() {
    assert(_anchor != null);
    _anchor!._closeChildren();
  }

  // ignore: use_setters_to_change_properties
  void _attach(_RawMenuAnchorState anchor) {
    _anchor = anchor;
  }

  void _detach(_RawMenuAnchorState anchor) {
    if (_anchor == anchor) {
      _anchor = null;
    }
  }

  /// Returns the [MenuController] of the ancestor [RawMenuAnchor] nearest to
  /// the given `context`, if one exists. Otherwise, returns null.
  ///
  /// If `createDependency` is true, then the provided [BuildContext] will
  /// rebuild when the menu opens and closes, or when the [MenuController]
  /// changes.
  static MenuController? maybeOf(BuildContext context, {bool createDependency = false}) {
    if (createDependency) {
      return context.dependOnInheritedWidgetOfExactType<_RawMenuAnchorScope>()?.controller;
    } else {
      return context.getInheritedWidgetOfExactType<_RawMenuAnchorScope>()?.controller;
    }
  }

  @override
  String toString() => describeIdentity(this);
}

// A widget that defines the menu drawn in the overlay.
class _MenuOverlay extends StatelessWidget {
  const _MenuOverlay({
    required this.alignmentOffset,
    required this.alignment,
    required this.menuAlignment,
    required this.position,
    required this.padding,
    this.semanticLabel,
    this.consumeOutsideTaps = true,
    required this.child,
  });

  final Offset alignmentOffset;
  final RawMenuAnchorOverlayPosition position;
  final Widget child;
  final EdgeInsetsGeometry? padding;
  final bool consumeOutsideTaps;
  final AlignmentGeometry? alignment;
  final AlignmentGeometry? menuAlignment;
  final String? semanticLabel;

  static final Map<Type, Action<Intent>> _defaultOverlayActions = <Type, Action<Intent>>{
    DirectionalFocusIntent: _OverlayDirectionalFocusAction(),
    _FocusFirstMenuItemIntent: _FocusFirstMenuItemAction(),
    _FocusLastMenuItemIntent: _FocusLastMenuItemAction(),
  };

  @override
  Widget build(BuildContext context) {
    final MenuController menuController = MenuController.maybeOf(context)!;
    final _RawMenuAnchorOverlayState anchor = menuController._anchor! as _RawMenuAnchorOverlayState;

    final Widget panel = Semantics.fromProperties(
      explicitChildNodes: true,
      properties: SemanticsProperties(scopesRoute: true, label: semanticLabel),
      child: TapRegion(
        groupId: position.tapRegionGroupId,
        consumeOutsideTaps: consumeOutsideTaps,
        onTapOutside: (PointerDownEvent event) {
          menuController.close();
        },
        child: FocusScope(
          node: anchor._menuScopeNode,
          skipTraversal: true,
          descendantsAreFocusable: true,
          child: Actions(
            actions: _defaultOverlayActions,
            child: Shortcuts(shortcuts: _kMenuTraversalShortcuts, child: child),
          ),
        ),
      ),
    );

    return ConstrainedBox(
      constraints: BoxConstraints.loose(position.overlaySize),
      child: Builder(
        builder: (BuildContext context) {
          final MediaQueryData mediaQuery = MediaQuery.of(context);
          final TextDirection textDirection = Directionality.of(context);
          // Resolve fallback alignment here so that alignmentOffset defaults to
          // being directionally-agnostic.
          final AlignmentGeometry anchorAlignment =
              alignment ??
              (anchor._isRootAnchor
                      ? AlignmentDirectional.bottomStart
                      : AlignmentDirectional.topEnd)
                  .resolve(textDirection);
          return CustomSingleChildLayout(
            delegate: _MenuLayout(
              screenPadding: mediaQuery.padding,
              padding: padding,
              avoidBounds: DisplayFeatureSubScreen.avoidBounds(mediaQuery).toSet(),
              textDirection: textDirection,
              anchorRect: position.anchorRect,
              alignmentOffset: alignmentOffset,
              menuPosition: position.position,
              menuAlignment: menuAlignment ?? AlignmentDirectional.topStart,
              alignment: anchorAlignment,
            ),
            child: panel,
          );
        },
      ),
    );
  }
}

/// An action that closes all the menus associated with the given
/// [MenuController].
///
/// See also:
///
///  * [RawMenuAnchor], a widget that hosts a cascading submenu.
///  * [MenuController], a controller used to manage a menu created by a
///    [RawMenuAnchor].
///  * [MenuBar], a widget that defines a menu bar with cascading submenus.
class DismissMenuAction extends DismissAction {
  /// Creates a [DismissMenuAction].
  DismissMenuAction({required this.controller});

  /// The [MenuController] that manages the menu which should be dismissed upon
  /// invocation.
  final MenuController controller;

  @override
  void invoke(DismissIntent intent) {
    controller._anchor!._root._close();
  }

  @override
  bool isEnabled(DismissIntent intent) {
    return controller._anchor != null;
  }
}

class _AnchorDirectionalFocusAction extends ContextAction<DirectionalFocusIntent> {
  _AnchorDirectionalFocusAction();

  @override
  void invoke(DirectionalFocusIntent intent, [BuildContext? context]) {
    final _RawMenuAnchorState? anchor = MenuController.maybeOf(context!)?._anchor;
    if (anchor is! _RawMenuAnchorOverlayState) {
      assert(
        anchor is! _RawMenuAnchorNodeState,
        'Menu panels should not invoke $_OverlayDirectionalFocusAction : $anchor',
      );
      primaryFocus?.focusInDirection(intent.direction);
      return;
    }

    final FocusNode? firstFocus = anchor._firstFocus;
    final FocusNode? lastFocus = anchor._lastFocus;
    switch (intent.direction) {
      case TraversalDirection.left:
      case TraversalDirection.right:
        break;
      case TraversalDirection.up:
        if (lastFocus != null) {
          return anchor._overlayTraversalPolicy?.requestFocusCallback(lastFocus);
        }
      case TraversalDirection.down:
        if (firstFocus != null) {
          return anchor._overlayTraversalPolicy?.requestFocusCallback(firstFocus);
        }
    }

    primaryFocus?.focusInDirection(intent.direction);
  }
}

class _OverlayDirectionalFocusAction extends ContextAction<DirectionalFocusIntent> {
  _OverlayDirectionalFocusAction();

  @override
  void invoke(DirectionalFocusIntent intent, [BuildContext? context]) {
    final _RawMenuAnchorState? anchor = MenuController.maybeOf(context!)?._anchor;
    if (anchor is! _RawMenuAnchorOverlayState) {
      assert(
        anchor is! _RawMenuAnchorNodeState,
        'Menu panels should not invoke $_OverlayDirectionalFocusAction : $anchor',
      );
      primaryFocus?.focusInDirection(intent.direction);
      return;
    }

    final bool isAnchorFocused = !(anchor._menuScopeNode?.hasFocus ?? false);
    _RawMenuAnchorOverlayState overlay = anchor;
    bool isSubmenuAnchor = false;

    // If we are an anchor in an overlay, switch to our parent anchor to move
    // between our siblings rather than the children in our overlay.
    if (isAnchorFocused && !anchor._isRootAnchor) {
      overlay = anchor._parent! as _RawMenuAnchorOverlayState;
      isSubmenuAnchor = true;
    }

    final FocusNode? firstFocus = overlay._firstFocus;
    final FocusNode? lastFocus = overlay._lastFocus;
    final TextDirection textDirection = Directionality.of(context);
    switch ((intent.direction, textDirection)) {
      case (TraversalDirection.up, _):
        if (lastFocus?.context == null) {
          break;
        }

        if (primaryFocus == lastFocus!.enclosingScope || primaryFocus == firstFocus) {
          overlay._overlayTraversalPolicy?.requestFocusCallback(lastFocus);
          return;
        }
      case (TraversalDirection.down, _):
        if (firstFocus?.context == null) {
          break;
        }

        if (primaryFocus == firstFocus!.enclosingScope || primaryFocus == lastFocus) {
          overlay._overlayTraversalPolicy?.requestFocusCallback(firstFocus);
          return;
        }
      case (TraversalDirection.left, TextDirection.ltr):
      case (TraversalDirection.right, TextDirection.rtl):
        if (isSubmenuAnchor) {
          if (anchor._isOpen) {
            anchor._close();
          } else if (anchor._parent?._parent != null) {
            anchor._parent?._close();
          }
          return;
        } else if (!anchor._isRootAnchor) {
          // When the anchor closes, focus will move to the parent anchor.
          anchor._close();
          return;
        }
      case (TraversalDirection.left, TextDirection.rtl):
      case (TraversalDirection.right, TextDirection.ltr):
        if (isSubmenuAnchor) {
          if (anchor._isOpen) {
            // Use requestFocusCallback to trigger scroll-to-focus behavior.
            anchor._overlayTraversalPolicy?.requestFocusCallback(anchor._firstFocus!);
          } else {
            anchor._open();
            SchedulerBinding.instance.addPostFrameCallback((Duration timestamp) {
              if (anchor._isOpen) {
                anchor._overlayTraversalPolicy?.requestFocusCallback(anchor._firstFocus!);
              }
            });
          }
          return;
        }
    }

    primaryFocus?.focusInDirection(intent.direction);
  }
}

class _FocusFirstMenuItemIntent extends Intent {
  const _FocusFirstMenuItemIntent();
}

class _FocusFirstMenuItemAction extends ContextAction<_FocusFirstMenuItemIntent> {
  _FocusFirstMenuItemAction();

  @override
  void invoke(_FocusFirstMenuItemIntent intent, [BuildContext? context]) {
    _RawMenuAnchorState? anchor = MenuController.maybeOf(context!)?._anchor;
    if (anchor is! _RawMenuAnchorOverlayState) {
      assert(
        anchor is! _RawMenuAnchorNodeState,
        'Menu panels should not invoke $_FocusFirstMenuItemAction : $anchor',
      );
      return;
    }

    final bool isAnchorFocused = !(anchor._menuScopeNode?.hasFocus ?? false);

    // If we are an anchor in an overlay, switch to our parent anchor to move
    // between our siblings rather than the children in our overlay.
    if (isAnchorFocused && !anchor._isRootAnchor) {
      anchor = anchor._parent! as _RawMenuAnchorOverlayState;
    }

    final FocusNode? firstFocus = anchor._firstFocus;
    if (firstFocus == null) {
      return;
    }

    anchor._overlayTraversalPolicy?.requestFocusCallback(firstFocus);
  }
}

class _FocusLastMenuItemIntent extends Intent {
  const _FocusLastMenuItemIntent();
}

class _FocusLastMenuItemAction extends ContextAction<_FocusLastMenuItemIntent> {
  _FocusLastMenuItemAction();

  @override
  void invoke(_FocusLastMenuItemIntent intent, [BuildContext? context]) {
    _RawMenuAnchorState? anchor = MenuController.maybeOf(context!)?._anchor;
    if (anchor is! _RawMenuAnchorOverlayState) {
      assert(
        anchor is! _RawMenuAnchorNodeState,
        'Menu panels should not invoke $_FocusFirstMenuItemAction : $anchor',
      );
      return;
    }

    final bool isAnchorFocused = !(anchor._menuScopeNode?.hasFocus ?? false);
    final bool inOverlay = !anchor._isRootAnchor;

    // If we are an anchor in an overlay, switch to our parent anchor to move
    // between our siblings rather than the children in our overlay.
    if (isAnchorFocused && inOverlay) {
      anchor = anchor._parent! as _RawMenuAnchorOverlayState;
    }

    final FocusNode? lastFocus = anchor._lastFocus;
    if (lastFocus == null) {
      return;
    }

    final FocusTraversalPolicy traversalPolicy =
        FocusTraversalGroup.maybeOfNode(lastFocus) ?? ReadingOrderTraversalPolicy();

    traversalPolicy.requestFocusCallback(lastFocus);
  }
}

// A layout delegate that positions the menu relative to its anchor.
class _MenuLayout extends SingleChildLayoutDelegate {
  const _MenuLayout({
    required this.alignmentOffset,
    required this.anchorRect,
    required this.screenPadding,
    required this.avoidBounds,
    required this.alignment,
    required this.menuAlignment,
    required this.textDirection,
    required EdgeInsetsGeometry? padding,
    this.menuPosition,
  }) : menuPadding = padding;

  // Rectangle of the button anchoring the menu overlay.
  final ui.Rect anchorRect;

  // The offset from the alignment position to find the ideal location for the
  // menu.
  final ui.Offset alignmentOffset;

  // The offset of the menu relative to the top-left corner of the anchor.
  final ui.Offset? menuPosition;

  // The padding obtained from calling [MediaQuery.paddingOf].
  //
  // Used to prevent the menu from being obstructed by system UI.
  final EdgeInsets screenPadding;

  // Padding applied to the menu surface.
  final EdgeInsetsGeometry? menuPadding;

  // List of rectangles that the menu should not overlap. Unusable screen area.
  final Set<Rect> avoidBounds;

  // The alignment of the menu attachment point relative to the anchor button.
  final AlignmentGeometry alignment;

  // The alignment of the menu attachment point relative to the menu surface.
  final AlignmentGeometry menuAlignment;

  // The direction in which the text flows within the menu.
  final ui.TextDirection textDirection;

  // Finds the closest screen to the anchor position.
  //
  // The closest screen is defined as the screen whose center is closest to the
  // anchor position.
  Rect _findClosestScreen(Size parentSize, Offset point, Set<Rect> avoidBounds) {
    final Iterable<ui.Rect> screens = DisplayFeatureSubScreen.subScreensInBounds(
      Offset.zero & parentSize,
      avoidBounds,
    );

    Rect closest = screens.first;
    for (final ui.Rect screen in screens.skip(1)) {
      if ((screen.center - point).distanceSquared < (closest.center - point).distanceSquared) {
        closest = screen;
      }
    }

    return closest;
  }

  Offset _fitInsideScreen(Rect screen, Size childSize, Offset position, Offset anchorPosition) {
    final EdgeInsets? padding = menuPadding?.resolve(textDirection);
    final Rect anchor = menuPosition == null ? anchorRect : anchorPosition & Size.zero;

    double x = position.dx;
    double y = position.dy;

    bool overLeftEdge(double x) => x < screen.left;
    bool overRightEdge(double x) => x > screen.right - childSize.width;
    bool overTopEdge(double y) => y < screen.top;
    bool overBottomEdge(double y) => y > screen.bottom - childSize.height;

    // Layout horizontally first to determine if the menu can be placed on
    // either side of the anchor without overlapping.
    bool hasHorizontalAnchorOverlap = childSize.width >= screen.width;
    if (hasHorizontalAnchorOverlap) {
      x = screen.left;
    } else {
      // Shift the menu left or right to adjust for padding.
      double? shiftX;
      if (padding != null && padding.horizontal > 0) {
        double ratio = (x - anchorPosition.dx) / childSize.width;
        ratio = ui.clampDouble(ratio, -1, 0);
        shiftX = padding.right * ratio + padding.left * (ratio + 1);
        x -= shiftX;
      }

      if (overLeftEdge(x)) {
        // Flip the X position across the horizontal midpoint of the anchor so that the menu is to the right of the anchor.
        double flipX = anchor.center.dx * 2 - position.dx - childSize.width;
        if (shiftX != null) {
          flipX -= padding!.horizontal + shiftX;
        }

        hasHorizontalAnchorOverlap = overRightEdge(flipX);
        if (hasHorizontalAnchorOverlap || overLeftEdge(flipX)) {
          x = screen.left;
        } else {
          x = flipX;
        }
      } else if (overRightEdge(x)) {
        // Flip the X position across the horizontal midpoint of the anchor so that the menu is to the left of the anchor.
        double flipX = anchor.center.dx * 2 - position.dx - childSize.width;
        if (shiftX != null) {
          flipX += padding!.horizontal - shiftX;
        }

        hasHorizontalAnchorOverlap = overLeftEdge(flipX);
        if (hasHorizontalAnchorOverlap || overRightEdge(flipX)) {
          x = screen.right - childSize.width;
        } else {
          x = flipX;
        }
      }
    }

    if (childSize.height >= screen.height) {
      // Menu is too big to fit on screen. Fit as much as possible.
      return Offset(x, screen.top);
    }

    if (hasHorizontalAnchorOverlap && !anchor.isEmpty) {
      // If both horizontal screen edges overlap, shift the menu upwards or
      // downwards by the minimum amount needed to avoid overlapping the anchor.
      //
      // NOTE: Menus that are deliberately overlapping the anchor will stop
      // overlapping the anchor, but only when the screen is very small.
      final double below = anchor.bottom - y;
      final double above = y + childSize.height - anchor.top;
      if (below > 0 && above > 0) {
        if (below > above) {
          y = anchor.top - childSize.height;
        } else {
          y = anchor.bottom;
        }
      }
    }

    // Remove vertical padding from the y component.
    double? shiftY;
    if (padding != null && padding.vertical > 0) {
      double ratio = (y - anchorPosition.dy) / childSize.height;
      ratio = ui.clampDouble(ratio, -1, 0);
      shiftY = padding.bottom * ratio + padding.top * (ratio + 1);
      y -= shiftY;
    }

    if (overTopEdge(y)) {
      // Flip the Y position across the vertical midpoint of the anchor so that the menu is below the anchor.
      double flipY = anchor.center.dy * 2 - position.dy - childSize.height;
      if (shiftY != null) {
        flipY -= padding!.vertical + shiftY;
      }

      if (overTopEdge(flipY) || overBottomEdge(flipY)) {
        y = screen.top;
      } else {
        y = flipY;
      }
    } else if (overBottomEdge(y)) {
      // Flip the Y position across the vertical midpoint of the anchor so that
      // the menu is above the anchor.
      double flipY = anchor.center.dy * 2 - position.dy - childSize.height;
      if (shiftY != null) {
        flipY += padding!.vertical - shiftY;
      }

      if (overTopEdge(flipY) || overBottomEdge(flipY)) {
        y = screen.bottom - childSize.height;
      } else {
        y = flipY;
      }
    }

    return Offset(x, y);
  }

  @override
  BoxConstraints getConstraintsForChild(BoxConstraints constraints) {
    // The menu can be at most the size of the overlay minus totalPadding.
    return BoxConstraints.loose(constraints.biggest).deflate(screenPadding);
  }

  @override
  Offset getPositionForChild(Size size, Size childSize) {
    // Point on the anchor where the menu is attached.
    Offset anchorOffset;
    if (menuPosition == null) {
      anchorOffset = alignment.resolve(textDirection).withinRect(anchorRect);
      anchorOffset += switch (textDirection) {
        ui.TextDirection.ltr => alignmentOffset,
        ui.TextDirection.rtl =>
          alignment is AlignmentDirectional
              ? Offset(-alignmentOffset.dx, alignmentOffset.dy)
              : alignmentOffset,
      };
    } else {
      anchorOffset = anchorRect.topLeft + menuPosition!;
    }

    final ui.Offset position =
        anchorOffset - menuAlignment.resolve(textDirection).alongSize(childSize);

    final Rect screen = _findClosestScreen(size, anchorRect.center, avoidBounds);

    return _fitInsideScreen(screenPadding.deflateRect(screen), childSize, position, anchorOffset);
  }

  @override
  bool shouldRelayout(_MenuLayout oldDelegate) {
    return anchorRect != oldDelegate.anchorRect ||
        alignment != oldDelegate.alignment ||
        alignmentOffset != oldDelegate.alignmentOffset ||
        menuAlignment != oldDelegate.menuAlignment ||
        menuPosition != oldDelegate.menuPosition ||
        menuPadding != oldDelegate.menuPadding ||
        screenPadding != oldDelegate.screenPadding ||
        textDirection != oldDelegate.textDirection ||
        !setEquals(avoidBounds, oldDelegate.avoidBounds);
  }
}

/// A debug print function, which should only be called within an assert, like
/// so:
///
///   assert(_debugMenuInfo('Debug Message'));
///
/// so that the call is entirely removed in release builds.
///
/// Enable debug printing by setting [_kDebugMenus] to true at the top of the
/// file.
bool _debugMenuInfo(String message, [Iterable<String>? details]) {
  assert(() {
    if (_kDebugMenus) {
      debugPrint('MENU: $message');
      if (details != null && details.isNotEmpty) {
        for (final String detail in details) {
          debugPrint('    $detail');
        }
      }
    }
    return true;
  }());
  // Return true so that it can be easily used inside of an assert.
  return true;
}
