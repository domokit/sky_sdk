// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/cupertino.dart';
///
/// @docImport 'app.dart';
/// @docImport 'checkbox_theme.dart';
/// @docImport 'dropdown_menu.dart';
/// @docImport 'radio_theme.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:flutter/widgets.dart';

import 'button_style.dart';
import 'button_style_button.dart';
import 'checkbox.dart';
import 'color_scheme.dart';
import 'colors.dart';
import 'constants.dart';
import 'icons.dart';
import 'ink_well.dart';
import 'material.dart';
import 'material_localizations.dart';
import 'material_state.dart';
import 'menu_bar_theme.dart';
import 'menu_button_theme.dart';
import 'menu_style.dart';
import 'menu_theme.dart';
import 'radio.dart';
import 'scrollbar.dart';
import 'text_button.dart';
import 'text_theme.dart';
import 'theme.dart';
import 'theme_data.dart';

// Examples can assume:
// bool _throwShotAway = false;
// late BuildContext context;
// enum SingingCharacter { lafayette }
// late SingingCharacter? _character;
// late StateSetter setState;

// Enable if you want verbose logging about menu changes.
const bool _kDebugMenus = false;

// The default size of the arrow in _MenuItemLabel that indicates that a menu
// has a submenu.
const double _kDefaultSubmenuIconSize = 24;

// The default spacing between the leading icon, label, trailing icon, and
// shortcut label in a _MenuItemLabel.
const double _kLabelItemDefaultSpacing = 12;

// The minimum spacing between the leading icon, label, trailing icon, and
// shortcut label in a _MenuItemLabel.
const double _kLabelItemMinSpacing = 4;

// Navigation shortcuts that we need to make sure are active when menus are
// open.
const Map<ShortcutActivator, Intent> _kMenuTraversalShortcuts = <ShortcutActivator, Intent>{
  SingleActivator(LogicalKeyboardKey.gameButtonA): ActivateIntent(),
  SingleActivator(LogicalKeyboardKey.escape): DismissIntent(),
  SingleActivator(LogicalKeyboardKey.tab): NextFocusIntent(),
  SingleActivator(LogicalKeyboardKey.tab, shift: true): PreviousFocusIntent(),
  SingleActivator(LogicalKeyboardKey.arrowDown): DirectionalFocusIntent(TraversalDirection.down),
  SingleActivator(LogicalKeyboardKey.arrowUp): DirectionalFocusIntent(TraversalDirection.up),
  SingleActivator(LogicalKeyboardKey.arrowLeft): DirectionalFocusIntent(TraversalDirection.left),
  SingleActivator(LogicalKeyboardKey.arrowRight): DirectionalFocusIntent(TraversalDirection.right),
};

// The minimum vertical spacing on the outside of menus.
const double _kMenuVerticalMinPadding = 8;

// How close to the edge of the safe area the menu will be placed.
const double _kMenuViewPadding = 8;

// The minimum horizontal spacing on the outside of the top level menu.
const double _kTopLevelMenuHorizontalMinPadding = 4;

/// The type of builder function used by [MenuAnchor.builder] to build the
/// widget that the [MenuAnchor] surrounds.
///
/// The `context` is the context that the widget is being built in.
///
/// The `controller` is the [MenuController] that can be used to open and close
/// the menu with.
///
/// The `child` is an optional child supplied as the [MenuAnchor.child]
/// attribute. The child is intended to be incorporated in the result of the
/// function.
typedef MenuAnchorChildBuilder =
    Widget Function(BuildContext context, MenuController controller, Widget? child);

/// A widget used to mark the "anchor" for a set of submenus, defining the
/// rectangle used to position the menu, which can be done either with an
/// explicit location, or with an alignment.
///
/// When creating a menu with [MenuBar] or a [SubmenuButton], a [MenuAnchor] is
/// not needed, since they provide their own internally.
///
/// The [MenuAnchor] is meant to be a slightly lower level interface than
/// [MenuBar], used in situations where a [MenuBar] isn't appropriate, or to
/// construct widgets or screen regions that have submenus.
///
/// {@tool dartpad}
/// This example shows how to use a [MenuAnchor] to wrap a button and open a
/// cascading menu from the button.
///
/// ** See code in examples/api/lib/material/menu_anchor/menu_anchor.0.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example shows how to use a [MenuAnchor] to create a cascading context
/// menu in a region of the view, positioned where the user clicks the mouse
/// with Ctrl pressed. The [anchorTapClosesMenu] attribute is set to true so
/// that clicks on the [MenuAnchor] area will cause the menus to be closed.
///
/// ** See code in examples/api/lib/material/menu_anchor/menu_anchor.1.dart **
/// {@end-tool}
///
/// {@tool dartpad}
/// This example demonstrates a simplified cascading menu using the [MenuAnchor]
/// widget.
///
/// ** See code in examples/api/lib/material/menu_anchor/menu_anchor.3.dart **
/// {@end-tool}
class MenuAnchor extends StatefulWidget {
  /// Creates a const [MenuAnchor].
  ///
  /// The [menuChildren] argument is required.
  const MenuAnchor({
    super.key,
    this.controller,
    this.childFocusNode,
    this.style,
    this.alignmentOffset = Offset.zero,
    this.layerLink,
    this.clipBehavior = Clip.hardEdge,
    @Deprecated(
      'Use consumeOutsideTap instead. '
      'This feature was deprecated after v3.16.0-8.0.pre.',
    )
    this.anchorTapClosesMenu = false,
    this.consumeOutsideTap = false,
    this.onOpen,
    this.onClose,
    this.crossAxisUnconstrained = true,
    required this.menuChildren,
    this.builder,
    this.child,
  });

  /// An optional controller that allows opening and closing of the menu from
  /// other widgets.
  final MenuController? controller;

  /// The [childFocusNode] attribute is the optional [FocusNode] also associated
  /// to the [child] or [builder] widget that opens the menu.
  ///
  /// The focus node should be attached to the widget that should receive focus
  /// if keyboard focus traversal moves the focus off of the submenu with the
  /// arrow keys.
  ///
  /// If not supplied, then keyboard traversal from the menu back to the
  /// controlling button when the menu is open is disabled.
  final FocusNode? childFocusNode;

  /// The [MenuStyle] that defines the visual attributes of the menu bar.
  ///
  /// Colors and sizing of the menus is controllable via the [MenuStyle].
  ///
  /// Defaults to the ambient [MenuThemeData.style].
  final MenuStyle? style;

  /// {@template flutter.material.MenuAnchor.alignmentOffset}
  /// The offset of the menu relative to the alignment origin determined by
  /// [MenuStyle.alignment] on the [style] attribute and the ambient
  /// [Directionality].
  ///
  /// Use this for adjustments of the menu placement.
  ///
  /// Increasing [Offset.dy] values of [alignmentOffset] move the menu position
  /// down.
  ///
  /// If the [MenuStyle.alignment] from [style] is not an [AlignmentDirectional]
  /// (e.g. [Alignment]), then increasing [Offset.dx] values of
  /// [alignmentOffset] move the menu position to the right.
  ///
  /// If the [MenuStyle.alignment] from [style] is an [AlignmentDirectional],
  /// then in a [TextDirection.ltr] [Directionality], increasing [Offset.dx]
  /// values of [alignmentOffset] move the menu position to the right. In a
  /// [TextDirection.rtl] directionality, increasing [Offset.dx] values of
  /// [alignmentOffset] move the menu position to the left.
  ///
  /// Defaults to [Offset.zero].
  /// {@endtemplate}
  final Offset? alignmentOffset;

  /// An optional [LayerLink] to attach the menu to the widget that this
  /// [MenuAnchor] surrounds.
  ///
  /// When provided, the menu will follow the widget that this [MenuAnchor]
  /// surrounds if it moves because of view insets changes.
  final LayerLink? layerLink;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.hardEdge].
  final Clip clipBehavior;

  /// Whether the menus will be closed if the anchor area is tapped.
  ///
  /// For menus opened by buttons that toggle the menu, if the button is tapped
  /// when the menu is open, the button should close the menu. But if
  /// [anchorTapClosesMenu] is true, then the menu will close, and
  /// (surprisingly) immediately re-open. This is because tapping on the button
  /// closes the menu before the `onPressed` or `onTap` handler is called
  /// because of it being considered to be "outside" the menu system, and then
  /// the button (seeing that the menu is closed) immediately reopens the menu.
  /// The result is that the user thinks that tapping on the button does
  /// nothing. So, for button-initiated menus, this value is typically false so
  /// that the menu anchor area is considered "inside" of the menu system and
  /// doesn't cause it to close unless [MenuController.close] is called.
  ///
  /// For menus that are positioned using [MenuController.open]'s `position`
  /// parameter, it is often desirable that clicking on the anchor always closes
  /// the menu since the anchor area isn't usually considered part of the menu
  /// system by the user. In this case [anchorTapClosesMenu] should be true.
  ///
  /// Defaults to false.
  @Deprecated(
    'Use consumeOutsideTap instead. '
    'This feature was deprecated after v3.16.0-8.0.pre.',
  )
  final bool anchorTapClosesMenu;

  /// Whether or not a tap event that closes the menu will be permitted to
  /// continue on to the gesture arena.
  ///
  /// If false, then tapping outside of a menu when the menu is open will both
  /// close the menu, and allow the tap to participate in the gesture arena. If
  /// true, then it will only close the menu, and the tap event will be
  /// consumed.
  ///
  /// Defaults to false.
  final bool consumeOutsideTap;

  /// A callback that is invoked when the menu is opened.
  final VoidCallback? onOpen;

  /// A callback that is invoked when the menu is closed.
  final VoidCallback? onClose;

  /// Determine if the menu panel can be wrapped by a [UnconstrainedBox] which allows
  /// the panel to render at its "natural" size.
  ///
  /// Defaults to true as it allows developers to render the menu panel at the
  /// size it should be. When it is set to false, it can be useful when the menu should
  /// be constrained in both main axis and cross axis, such as a [DropdownMenu].
  final bool crossAxisUnconstrained;

  /// A list of children containing the menu items that are the contents of the
  /// menu surrounded by this [MenuAnchor].
  ///
  /// {@macro flutter.material.MenuBar.shortcuts_note}
  final List<Widget> menuChildren;

  /// The widget that this [MenuAnchor] surrounds.
  ///
  /// Typically this is a button used to open the menu by calling
  /// [MenuController.open] on the `controller` passed to the builder.
  ///
  /// If not supplied, then the [MenuAnchor] will be the size that its parent
  /// allocates for it.
  ///
  /// If provided, the builder will be called each time the menu is opened or
  /// closed.
  final MenuAnchorChildBuilder? builder;

  /// The optional child to be passed to the [builder].
  ///
  /// Supply this child if there is a portion of the widget tree built in
  /// [builder] that doesn't depend on the `controller` or `context` supplied to
  /// the [builder]. It will be more efficient, since Flutter doesn't then need
  /// to rebuild this child when those change.
  final Widget? child;

  @override
  State<MenuAnchor> createState() => _MenuAnchorState();

  @override
  List<DiagnosticsNode> debugDescribeChildren() {
    return menuChildren.map<DiagnosticsNode>((Widget child) => child.toDiagnosticsNode()).toList();
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      FlagProperty('anchorTapClosesMenu', value: anchorTapClosesMenu, ifTrue: 'AUTO-CLOSE'),
    );
    properties.add(DiagnosticsProperty<FocusNode?>('focusNode', childFocusNode));
    properties.add(DiagnosticsProperty<MenuStyle?>('style', style));
    properties.add(EnumProperty<Clip>('clipBehavior', clipBehavior));
    properties.add(DiagnosticsProperty<Offset?>('alignmentOffset', alignmentOffset));
  }
}

class _MenuAnchorState extends State<MenuAnchor> {
  // This is the global key that is used later to determine the bounding rect
  // for the anchor's region that the CustomSingleChildLayout's delegate
  // uses to determine where to place the menu on the screen and to avoid the
  // view's edges.
  final GlobalKey<_MenuAnchorState> _anchorKey = GlobalKey<_MenuAnchorState>(
    debugLabel: kReleaseMode ? null : 'MenuAnchor',
  );
  _MenuAnchorState? _parent;
  late final FocusScopeNode _menuScopeNode;
  MenuController? _internalMenuController;
  final List<_MenuAnchorState> _anchorChildren = <_MenuAnchorState>[];
  ScrollPosition? _scrollPosition;
  Size? _viewSize;
  final OverlayPortalController _overlayController = OverlayPortalController(
    debugLabel: kReleaseMode ? null : 'MenuAnchor controller',
  );
  Offset? _menuPosition;
  Axis get _orientation => Axis.vertical;
  bool get _isOpen => _overlayController.isShowing;
  bool get _isRoot => _parent == null;
  MenuController get _menuController => widget.controller ?? _internalMenuController!;

  @override
  void initState() {
    super.initState();
    _menuScopeNode = FocusScopeNode(
      debugLabel: kReleaseMode ? null : '${describeIdentity(this)} Sub Menu',
    );
    if (widget.controller == null) {
      _internalMenuController = MenuController();
    }
    _menuController._attach(this);
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
    _menuScopeNode.dispose();
    super.dispose();
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    final _MenuAnchorState? newParent = _MenuAnchorState._maybeOf(context);
    if (newParent != _parent) {
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
  void didUpdateWidget(MenuAnchor oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.controller != widget.controller) {
      oldWidget.controller?._detach(this);
      if (widget.controller != null) {
        _internalMenuController?._detach(this);
        _internalMenuController = null;
        widget.controller?._attach(this);
      } else {
        assert(_internalMenuController == null);
        _internalMenuController = MenuController().._attach(this);
      }
    }
    assert(_menuController._anchor == this);
  }

  @override
  Widget build(BuildContext context) {
    Widget contents = _buildContents(context);
    if (widget.layerLink != null) {
      contents = CompositedTransformTarget(link: widget.layerLink!, child: contents);
    }

    Widget child = OverlayPortal(
      controller: _overlayController,
      overlayChildBuilder: (BuildContext context) {
        return _Submenu(
          anchor: this,
          layerLink: widget.layerLink,
          menuStyle: widget.style,
          alignmentOffset: widget.alignmentOffset ?? Offset.zero,
          menuPosition: _menuPosition,
          clipBehavior: widget.clipBehavior,
          menuChildren: widget.menuChildren,
          crossAxisUnconstrained: widget.crossAxisUnconstrained,
        );
      },
      child: contents,
    );

    if (!widget.anchorTapClosesMenu) {
      child = TapRegion(
        groupId: _root,
        consumeOutsideTaps: _root._isOpen && widget.consumeOutsideTap,
        onTapOutside: (PointerDownEvent event) {
          assert(_debugMenuInfo('Tapped Outside ${widget.controller}'));
          _closeChildren();
        },
        child: child,
      );
    }

    // This `Shortcuts` is needed so that shortcuts work when the focus is on
    // MenuAnchor (specifically, the root menu, since submenus have their own
    // `Shortcuts`).
    return Shortcuts(
      shortcuts: _kMenuTraversalShortcuts,
      // Ignore semantics here and since the same information is typically
      // also provided by the children.
      includeSemantics: false,
      child: _MenuAnchorScope(anchorKey: _anchorKey, anchor: this, isOpen: _isOpen, child: child),
    );
  }

  Widget _buildContents(BuildContext context) {
    return Actions(
      actions: <Type, Action<Intent>>{
        DismissIntent: DismissMenuAction(controller: _menuController),
      },
      child: Builder(
        key: _anchorKey,
        builder: (BuildContext context) {
          return widget.builder?.call(context, _menuController, widget.child) ??
              widget.child ??
              const SizedBox();
        },
      ),
    );
  }

  // Returns the first focusable item in the submenu, where "first" is
  // determined by the focus traversal policy.
  FocusNode? get _firstItemFocusNode {
    if (_menuScopeNode.context == null) {
      return null;
    }
    final FocusTraversalPolicy policy =
        FocusTraversalGroup.maybeOf(_menuScopeNode.context!) ?? ReadingOrderTraversalPolicy();
    return policy.findFirstFocus(_menuScopeNode, ignoreCurrentFocus: true);
  }

  FocusNode? get _lastItemFocusNode {
    if (_menuScopeNode.context == null) {
      return null;
    }
    final FocusTraversalPolicy policy =
        FocusTraversalGroup.maybeOf(_menuScopeNode.context!) ?? ReadingOrderTraversalPolicy();
    return policy.findLastFocus(_menuScopeNode, ignoreCurrentFocus: true);
  }

  void _addChild(_MenuAnchorState child) {
    assert(_isRoot || _debugMenuInfo('Added root child: $child'));
    assert(!_anchorChildren.contains(child));
    _anchorChildren.add(child);
    assert(_debugMenuInfo('Added:\n${child.widget.toStringDeep()}'));
    assert(_debugMenuInfo('Tree:\n${widget.toStringDeep()}'));
  }

  void _removeChild(_MenuAnchorState child) {
    assert(_isRoot || _debugMenuInfo('Removed root child: $child'));
    assert(_anchorChildren.contains(child));
    assert(_debugMenuInfo('Removing:\n${child.widget.toStringDeep()}'));
    _anchorChildren.remove(child);
    assert(_debugMenuInfo('Tree:\n${widget.toStringDeep()}'));
  }

  _MenuAnchorState get _root {
    _MenuAnchorState anchor = this;
    while (anchor._parent != null) {
      anchor = anchor._parent!;
    }
    return anchor;
  }

  void _childChangedOpenState() {
    _parent?._childChangedOpenState();
    assert(mounted);
    if (SchedulerBinding.instance.schedulerPhase != SchedulerPhase.persistentCallbacks) {
      setState(() {
        // Mark dirty now, but only if not in a build.
      });
    } else {
      SchedulerBinding.instance.addPostFrameCallback((Duration _) {
        setState(() {
          // Mark dirty after this frame, but only if in a build.
        });
      });
    }
  }

  void _focusButton() {
    if (widget.childFocusNode == null) {
      return;
    }
    assert(_debugMenuInfo('Requesting focus for ${widget.childFocusNode}'));
    widget.childFocusNode!.requestFocus();
  }

  void _handleScroll() {
    // If an ancestor scrolls, and we're a root anchor, then close the menus.
    // Don't just close it on *any* scroll, since we want to be able to scroll
    // menus themselves if they're too big for the view.
    if (_isRoot) {
      _close();
    }
  }

  /// Open the menu, optionally at a position relative to the [MenuAnchor].
  ///
  /// Call this when the menu should be shown to the user.
  ///
  /// The optional `position` argument will specify the location of the menu in
  /// the local coordinates of the [MenuAnchor], ignoring any
  /// [MenuStyle.alignment] and/or [MenuAnchor.alignmentOffset] that were
  /// specified.
  void _open({Offset? position}) {
    assert(_menuController._anchor == this);
    if (_isOpen && position == null) {
      assert(_debugMenuInfo("Not opening $this because it's already open"));
      return;
    }
    if (_isOpen && position != null) {
      // The menu is already open, but we need to move to another location, so
      // close it first.
      _close();
    }
    assert(
      _debugMenuInfo(
        'Opening $this at ${position ?? Offset.zero} with alignment offset ${widget.alignmentOffset ?? Offset.zero}',
      ),
    );
    _parent?._closeChildren(); // Close all siblings.
    assert(!_overlayController.isShowing);

    _parent?._childChangedOpenState();
    _menuPosition = position;
    _overlayController.show();

    if (_isRoot) {
      _focusButton();
    }

    widget.onOpen?.call();
    if (mounted && SchedulerBinding.instance.schedulerPhase != SchedulerPhase.persistentCallbacks) {
      setState(() {
        // Mark dirty to ensure UI updates
      });
    }
  }

  /// Close the menu.
  ///
  /// Call this when the menu should be closed. Has no effect if the menu is
  /// already closed.
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

  void _closeChildren({bool inDispose = false}) {
    assert(_debugMenuInfo('Closing children of $this${inDispose ? ' (dispose)' : ''}'));
    for (final _MenuAnchorState child in List<_MenuAnchorState>.from(_anchorChildren)) {
      child._close(inDispose: inDispose);
    }
  }

  // Returns the active anchor in the given context, if any, and creates a
  // dependency relationship that will rebuild the context when the node
  // changes.
  static _MenuAnchorState? _maybeOf(BuildContext context) {
    return context.dependOnInheritedWidgetOfExactType<_MenuAnchorScope>()?.anchor;
  }

  @override
  String toString({DiagnosticLevel minLevel = DiagnosticLevel.debug}) {
    return describeIdentity(this);
  }
}

/// A controller to manage a menu created by a [MenuBar] or [MenuAnchor].
///
/// A [MenuController] is used to control and interrogate a menu after it has
/// been created, with methods such as [open] and [close], and state accessors
/// like [isOpen].
///
/// See also:
///
/// * [MenuAnchor], a widget that defines a region that has submenu.
/// * [MenuBar], a widget that creates a menu bar, that can take an optional
///   [MenuController].
/// * [SubmenuButton], a widget that has a button that manages a submenu.
class MenuController {
  /// The anchor that this controller controls.
  ///
  /// This is set automatically when a [MenuController] is given to the anchor
  /// it controls.
  _MenuAnchorState? _anchor;

  /// Whether or not the associated menu is currently open.
  bool get isOpen {
    return _anchor?._isOpen ?? false;
  }

  /// Close the menu that this menu controller is associated with.
  ///
  /// Associating with a menu is done by passing a [MenuController] to a
  /// [MenuAnchor]. A [MenuController] is also be received by the
  /// [MenuAnchor.builder] when invoked.
  ///
  /// If the menu's anchor point (either a [MenuBar] or a [MenuAnchor]) is
  /// scrolled by an ancestor, or the view changes size, then any open menu will
  /// automatically close.
  void close() {
    _anchor?._close();
  }

  /// Opens the menu that this menu controller is associated with.
  ///
  /// If `position` is given, then the menu will open at the position given, in
  /// the coordinate space of the [MenuAnchor] this controller is attached to.
  ///
  /// If given, the `position` will override the [MenuAnchor.alignmentOffset]
  /// given to the [MenuAnchor].
  ///
  /// If the menu's anchor point (either a [MenuBar] or a [MenuAnchor]) is
  /// scrolled by an ancestor, or the view changes size, then any open menu will
  /// automatically close.
  void open({Offset? position}) {
    assert(_anchor != null);
    _anchor!._open(position: position);
  }

  // ignore: use_setters_to_change_properties
  void _attach(_MenuAnchorState anchor) {
    _anchor = anchor;
  }

  void _detach(_MenuAnchorState anchor) {
    if (_anchor == anchor) {
      _anchor = null;
    }
  }
}

/// A menu bar that manages cascading child menus.
///
/// This is a Material Design menu bar that typically resides above the main
/// body of an application (but can go anywhere) that defines a menu system for
/// invoking callbacks in response to user selection of a menu item.
///
/// The menus can be opened with a click or tap. Once a menu is opened, it can
/// be navigated by using the arrow and tab keys or via mouse hover. Selecting a
/// menu item can be done by pressing enter, or by clicking or tapping on the
/// menu item. Clicking or tapping on any part of the user interface that isn't
/// part of the menu system controlled by the same controller will cause all of
/// the menus controlled by that controller to close, as will pressing the
/// escape key.
///
/// When a menu item with a submenu is clicked on, it toggles the visibility of
/// the submenu. When the menu item is hovered over, the submenu will open, and
/// hovering over other items will close the previous menu and open the newly
/// hovered one. When those open/close transitions occur,
/// [SubmenuButton.onOpen], and [SubmenuButton.onClose] are called on the
/// corresponding [SubmenuButton] child of the menu bar.
///
/// {@template flutter.material.MenuBar.shortcuts_note}
/// Menus using [MenuItemButton] can have a [SingleActivator] or
/// [CharacterActivator] assigned to them as their [MenuItemButton.shortcut],
/// which will display an appropriate shortcut hint. Even though the shortcut
/// labels are displayed in the menu, shortcuts are not automatically handled.
/// They must be available in whatever context they are appropriate, and handled
/// via another mechanism.
///
/// If shortcuts should be generally enabled, but are not easily defined in a
/// context surrounding the menu bar, consider registering them with a
/// [ShortcutRegistry] (one is already included in the [WidgetsApp], and thus
/// also [MaterialApp] and [CupertinoApp]), as shown in the example below. To be
/// sure that selecting a menu item and triggering the shortcut do the same
/// thing, it is recommended that they call the same callback.
///
/// {@tool dartpad} This example shows a [MenuBar] that contains a single top
/// level menu, containing three items: "About", a checkbox menu item for
/// showing a message, and "Quit". The items are identified with an enum value,
/// and the shortcuts are registered globally with the [ShortcutRegistry].
///
/// ** See code in examples/api/lib/material/menu_anchor/menu_bar.0.dart **
/// {@end-tool}
/// {@endtemplate}
///
/// {@macro flutter.material.MenuAcceleratorLabel.accelerator_sample}
///
/// See also:
///
/// * [MenuAnchor], a widget that creates a region with a submenu and shows it
///   when requested.
/// * [SubmenuButton], a menu item which manages a submenu.
/// * [MenuItemButton], a leaf menu item which displays the label, an optional
///   shortcut label, and optional leading and trailing icons.
/// * [PlatformMenuBar], which creates a menu bar that is rendered by the host
///   platform instead of by Flutter (on macOS, for example).
/// * [ShortcutRegistry], a registry of shortcuts that apply for the entire
///   application.
/// * [VoidCallbackIntent], to define intents that will call a [VoidCallback] and
///   work with the [Actions] and [Shortcuts] system.
/// * [CallbackShortcuts], to define shortcuts that call a callback without
///   involving [Actions].
class MenuBar extends StatelessWidget {
  /// Creates a const [MenuBar].
  ///
  /// The [children] argument is required.
  const MenuBar({
    super.key,
    this.style,
    this.clipBehavior = Clip.none,
    this.controller,
    required this.children,
  });

  /// The [MenuStyle] that defines the visual attributes of the menu bar.
  ///
  /// Colors and sizing of the menus is controllable via the [MenuStyle].
  ///
  /// Defaults to the ambient [MenuThemeData.style].
  final MenuStyle? style;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.none].
  final Clip clipBehavior;

  /// The [MenuController] to use for this menu bar.
  final MenuController? controller;

  /// The list of menu items that are the top level children of the [MenuBar].
  ///
  /// A Widget in Flutter is immutable, so directly modifying the [children]
  /// with [List] APIs such as `someMenuBarWidget.menus.add(...)` will result in
  /// incorrect behaviors. Whenever the menus list is modified, a new list
  /// object must be provided.
  ///
  /// {@macro flutter.material.MenuBar.shortcuts_note}
  final List<Widget> children;

  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasOverlay(context));
    return _MenuBarAnchor(
      controller: controller,
      clipBehavior: clipBehavior,
      style: style,
      menuChildren: children,
    );
  }

  @override
  List<DiagnosticsNode> debugDescribeChildren() {
    return <DiagnosticsNode>[
      ...children.map<DiagnosticsNode>((Widget item) => item.toDiagnosticsNode()),
    ];
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<MenuStyle?>('style', style, defaultValue: null));
    properties.add(DiagnosticsProperty<Clip>('clipBehavior', clipBehavior, defaultValue: null));
  }
}

/// A button for use in a [MenuBar], in a menu created with [MenuAnchor], or on
/// its own, that can be activated by click or keyboard navigation.
///
/// This widget represents a leaf entry in a menu hierarchy that is typically
/// part of a [MenuBar], but may be used independently, or as part of a menu
/// created with a [MenuAnchor].
///
/// {@macro flutter.material.MenuBar.shortcuts_note}
///
/// See also:
///
/// * [MenuBar], a class that creates a top level menu bar in a Material Design
///   style.
/// * [MenuAnchor], a widget that creates a region with a submenu and shows it
///   when requested.
/// * [SubmenuButton], a menu item similar to this one which manages a submenu.
/// * [PlatformMenuBar], which creates a menu bar that is rendered by the host
///   platform instead of by Flutter (on macOS, for example).
/// * [ShortcutRegistry], a registry of shortcuts that apply for the entire
///   application.
/// * [VoidCallbackIntent], to define intents that will call a [VoidCallback] and
///   work with the [Actions] and [Shortcuts] system.
/// * [CallbackShortcuts] to define shortcuts that call a callback without
///   involving [Actions].
class MenuItemButton extends StatefulWidget {
  /// Creates a const [MenuItemButton].
  ///
  /// The [child] attribute is required.
  const MenuItemButton({
    super.key,
    this.onPressed,
    this.onHover,
    this.requestFocusOnHover = true,
    this.onFocusChange,
    this.focusNode,
    this.autofocus = false,
    this.shortcut,
    this.semanticsLabel,
    this.style,
    this.statesController,
    this.clipBehavior = Clip.none,
    this.leadingIcon,
    this.trailingIcon,
    this.closeOnActivate = true,
    this.overflowAxis = Axis.horizontal,
    this.child,
  });

  /// Called when the button is tapped or otherwise activated.
  ///
  /// If this callback is null, then the button will be disabled.
  ///
  /// See also:
  ///
  ///  * [enabled], which is true if the button is enabled.
  final VoidCallback? onPressed;

  /// Called when a pointer enters or exits the button response area.
  ///
  /// The value passed to the callback is true if a pointer has entered button
  /// area and false if a pointer has exited.
  final ValueChanged<bool>? onHover;

  /// Determine if hovering can request focus.
  ///
  /// Defaults to true.
  final bool requestFocusOnHover;

  /// Handler called when the focus changes.
  ///
  /// Called with true if this widget's node gains focus, and false if it loses
  /// focus.
  final ValueChanged<bool>? onFocusChange;

  /// {@macro flutter.widgets.Focus.focusNode}
  final FocusNode? focusNode;

  /// {@macro flutter.widgets.Focus.autofocus}
  final bool autofocus;

  /// The optional shortcut that selects this [MenuItemButton].
  ///
  /// {@macro flutter.material.MenuBar.shortcuts_note}
  final MenuSerializableShortcut? shortcut;

  /// An optional Semantics label, applied to the entire [MenuItemButton].
  ///
  /// A screen reader will default to reading the derived text on the
  /// [MenuItemButton] itself, which is not guaranteed to be readable.
  /// (For some shortcuts, such as comma, semicolon, and other
  /// punctuation, screen readers read silence).
  ///
  /// Setting this label overwrites the semantics properties of the entire
  /// Widget, including its children. Consider wrapping this widget in
  /// [Semantics] if you want to customize other properties besides just
  /// the label.
  ///
  /// Null by default.
  final String? semanticsLabel;

  /// Customizes this button's appearance.
  ///
  /// Non-null properties of this style override the corresponding properties in
  /// [themeStyleOf] and [defaultStyleOf]. [WidgetStateProperty]s that resolve
  /// to non-null values will similarly override the corresponding
  /// [WidgetStateProperty]s in [themeStyleOf] and [defaultStyleOf].
  ///
  /// Null by default.
  final ButtonStyle? style;

  /// {@macro flutter.material.inkwell.statesController}
  final MaterialStatesController? statesController;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.none].
  final Clip clipBehavior;

  /// An optional icon to display before the [child] label.
  final Widget? leadingIcon;

  /// An optional icon to display after the [child] label.
  final Widget? trailingIcon;

  /// {@template flutter.material.menu_anchor.closeOnActivate}
  /// Determines if the menu will be closed when a [MenuItemButton]
  /// is pressed.
  ///
  /// Defaults to true.
  /// {@endtemplate}
  final bool closeOnActivate;

  /// The direction in which the menu item expands.
  ///
  /// If the menu item button is a descendent of [MenuAnchor] or [MenuBar], then
  /// this property is ignored.
  ///
  /// If [overflowAxis] is [Axis.vertical], the menu will be expanded vertically.
  /// If [overflowAxis] is [Axis.horizontal], then the menu will be
  /// expanded horizontally.
  ///
  /// Defaults to [Axis.horizontal].
  final Axis overflowAxis;

  /// The widget displayed in the center of this button.
  ///
  /// Typically this is the button's label, using a [Text] widget.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// Whether the button is enabled or disabled.
  ///
  /// To enable a button, set its [onPressed] property to a non-null value.
  bool get enabled => onPressed != null;

  @override
  State<MenuItemButton> createState() => _MenuItemButtonState();

  /// Defines the button's default appearance.
  ///
  /// {@macro flutter.material.text_button.default_style_of}
  ///
  /// {@macro flutter.material.text_button.material3_defaults}
  ButtonStyle defaultStyleOf(BuildContext context) {
    return _MenuButtonDefaultsM3(context);
  }

  /// Returns the [MenuButtonThemeData.style] of the closest
  /// [MenuButtonTheme] ancestor.
  ButtonStyle? themeStyleOf(BuildContext context) {
    return MenuButtonTheme.of(context).style;
  }

  /// A static convenience method that constructs a [MenuItemButton]'s
  /// [ButtonStyle] given simple values.
  ///
  /// The [foregroundColor] color is used to create a [WidgetStateProperty]
  /// [ButtonStyle.foregroundColor] value. Specify a value for [foregroundColor]
  /// to specify the color of the button's icons. Use [backgroundColor] for the
  /// button's background fill color. Use [disabledForegroundColor] and
  /// [disabledBackgroundColor] to specify the button's disabled icon and fill
  /// color.
  ///
  /// Similarly, the [enabledMouseCursor] and [disabledMouseCursor]
  /// parameters are used to construct [ButtonStyle.mouseCursor].
  ///
  /// The [iconColor], [disabledIconColor] are used to construct
  /// [ButtonStyle.iconColor] and [iconSize] is used to construct
  /// [ButtonStyle.iconSize].
  ///
  /// All of the other parameters are either used directly or used to create a
  /// [WidgetStateProperty] with a single value for all states.
  ///
  /// All parameters default to null, by default this method returns a
  /// [ButtonStyle] that doesn't override anything.
  ///
  /// For example, to override the default foreground color for a
  /// [MenuItemButton], as well as its overlay color, with all of the standard
  /// opacity adjustments for the pressed, focused, and hovered states, one
  /// could write:
  ///
  /// ```dart
  /// MenuItemButton(
  ///   leadingIcon: const Icon(Icons.pets),
  ///   style: MenuItemButton.styleFrom(foregroundColor: Colors.green),
  ///   onPressed: () {
  ///     // ...
  ///   },
  ///   child: const Text('Button Label'),
  /// ),
  /// ```
  static ButtonStyle styleFrom({
    Color? foregroundColor,
    Color? backgroundColor,
    Color? disabledForegroundColor,
    Color? disabledBackgroundColor,
    Color? shadowColor,
    Color? surfaceTintColor,
    Color? iconColor,
    double? iconSize,
    Color? disabledIconColor,
    TextStyle? textStyle,
    Color? overlayColor,
    double? elevation,
    EdgeInsetsGeometry? padding,
    Size? minimumSize,
    Size? fixedSize,
    Size? maximumSize,
    MouseCursor? enabledMouseCursor,
    MouseCursor? disabledMouseCursor,
    BorderSide? side,
    OutlinedBorder? shape,
    VisualDensity? visualDensity,
    MaterialTapTargetSize? tapTargetSize,
    Duration? animationDuration,
    bool? enableFeedback,
    AlignmentGeometry? alignment,
    InteractiveInkFeatureFactory? splashFactory,
  }) {
    return TextButton.styleFrom(
      foregroundColor: foregroundColor,
      backgroundColor: backgroundColor,
      disabledBackgroundColor: disabledBackgroundColor,
      disabledForegroundColor: disabledForegroundColor,
      shadowColor: shadowColor,
      surfaceTintColor: surfaceTintColor,
      iconColor: iconColor,
      iconSize: iconSize,
      disabledIconColor: disabledIconColor,
      textStyle: textStyle,
      overlayColor: overlayColor,
      elevation: elevation,
      padding: padding,
      minimumSize: minimumSize,
      fixedSize: fixedSize,
      maximumSize: maximumSize,
      enabledMouseCursor: enabledMouseCursor,
      disabledMouseCursor: disabledMouseCursor,
      side: side,
      shape: shape,
      visualDensity: visualDensity,
      tapTargetSize: tapTargetSize,
      animationDuration: animationDuration,
      enableFeedback: enableFeedback,
      alignment: alignment,
      splashFactory: splashFactory,
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(FlagProperty('enabled', value: onPressed != null, ifFalse: 'DISABLED'));
    properties.add(DiagnosticsProperty<ButtonStyle?>('style', style, defaultValue: null));
    properties.add(
      DiagnosticsProperty<MenuSerializableShortcut?>('shortcut', shortcut, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<FocusNode?>('focusNode', focusNode, defaultValue: null));
    properties.add(EnumProperty<Clip>('clipBehavior', clipBehavior, defaultValue: Clip.none));
    properties.add(
      DiagnosticsProperty<MaterialStatesController?>(
        'statesController',
        statesController,
        defaultValue: null,
      ),
    );
  }
}

class _MenuItemButtonState extends State<MenuItemButton> {
  // If a focus node isn't given to the widget, then we have to manage our own.
  FocusNode? _internalFocusNode;
  FocusNode get _focusNode => widget.focusNode ?? _internalFocusNode!;
  _MenuAnchorState? get _anchor => _MenuAnchorState._maybeOf(context);
  bool _isHovered = false;

  @override
  void initState() {
    super.initState();
    _createInternalFocusNodeIfNeeded();
    _focusNode.addListener(_handleFocusChange);
  }

  @override
  void dispose() {
    _focusNode.removeListener(_handleFocusChange);
    _internalFocusNode?.dispose();
    _internalFocusNode = null;
    super.dispose();
  }

  @override
  void didUpdateWidget(MenuItemButton oldWidget) {
    if (widget.focusNode != oldWidget.focusNode) {
      (oldWidget.focusNode ?? _internalFocusNode)?.removeListener(_handleFocusChange);
      if (widget.focusNode != null) {
        _internalFocusNode?.dispose();
        _internalFocusNode = null;
      }
      _createInternalFocusNodeIfNeeded();
      _focusNode.addListener(_handleFocusChange);
    }
    super.didUpdateWidget(oldWidget);
  }

  @override
  Widget build(BuildContext context) {
    // Since we don't want to use the theme style or default style from the
    // TextButton, we merge the styles, merging them in the right order when
    // each type of style exists. Each "*StyleOf" function is only called once.
    ButtonStyle mergedStyle =
        widget.themeStyleOf(context)?.merge(widget.defaultStyleOf(context)) ??
        widget.defaultStyleOf(context);
    if (widget.style != null) {
      mergedStyle = widget.style!.merge(mergedStyle);
    }

    Widget child = TextButton(
      onPressed: widget.enabled ? _handleSelect : null,
      onFocusChange: widget.enabled ? widget.onFocusChange : null,
      focusNode: _focusNode,
      style: mergedStyle,
      autofocus: widget.enabled && widget.autofocus,
      statesController: widget.statesController,
      clipBehavior: widget.clipBehavior,
      isSemanticButton: null,
      child: _MenuItemLabel(
        leadingIcon: widget.leadingIcon,
        shortcut: widget.shortcut,
        semanticsLabel: widget.semanticsLabel,
        trailingIcon: widget.trailingIcon,
        hasSubmenu: false,
        overflowAxis: _anchor?._orientation ?? widget.overflowAxis,
        child: widget.child,
      ),
    );

    if (_platformSupportsAccelerators && widget.enabled) {
      child = MenuAcceleratorCallbackBinding(onInvoke: _handleSelect, child: child);
    }

    if (widget.onHover != null || widget.requestFocusOnHover) {
      child = MouseRegion(onHover: _handlePointerHover, onExit: _handlePointerExit, child: child);
    }

    return MergeSemantics(child: child);
  }

  void _handleFocusChange() {
    if (!_focusNode.hasPrimaryFocus) {
      // Close any child menus of this button's menu.
      _MenuAnchorState._maybeOf(context)?._closeChildren();
    }
  }

  void _handlePointerExit(PointerExitEvent event) {
    if (_isHovered) {
      widget.onHover?.call(false);
      _isHovered = false;
    }
  }

  // TextButton.onHover and MouseRegion.onHover can't be used without triggering
  // focus on scroll.
  void _handlePointerHover(PointerHoverEvent event) {
    if (!_isHovered) {
      _isHovered = true;
      widget.onHover?.call(true);
      if (widget.requestFocusOnHover) {
        assert(_debugMenuInfo('Requesting focus for $_focusNode from hover'));
        _focusNode.requestFocus();

        // Without invalidating the focus policy, switching to directional focus
        // may not originate at this node.
        FocusTraversalGroup.of(context).invalidateScopeData(FocusScope.of(context));
      }
    }
  }

  void _handleSelect() {
    assert(_debugMenuInfo('Selected ${widget.child} menu'));
    if (widget.closeOnActivate) {
      _MenuAnchorState._maybeOf(context)?._root._close();
    }
    // Delay the call to onPressed until post-frame so that the focus is
    // restored to what it was before the menu was opened before the action is
    // executed.
    SchedulerBinding.instance.addPostFrameCallback((Duration _) {
      FocusManager.instance.applyFocusChangesIfNeeded();
      widget.onPressed?.call();
    }, debugLabel: 'MenuAnchor.onPressed');
  }

  void _createInternalFocusNodeIfNeeded() {
    if (widget.focusNode == null) {
      _internalFocusNode = FocusNode();
      assert(() {
        _internalFocusNode?.debugLabel = '$MenuItemButton(${widget.child})';
        return true;
      }());
    }
  }
}

/// A menu item that combines a [Checkbox] widget with a [MenuItemButton].
///
/// To style the checkbox separately from the button, add a [CheckboxTheme]
/// ancestor.
///
/// {@tool dartpad}
/// This example shows a menu with a checkbox that shows a message in the body
/// of the app if checked.
///
/// ** See code in examples/api/lib/material/menu_anchor/checkbox_menu_button.0.dart **
/// {@end-tool}
///
/// See also:
///
/// - [MenuBar], a widget that creates a menu bar of cascading menu items.
/// - [MenuAnchor], a widget that defines a region which can host a cascading
///   menu.
class CheckboxMenuButton extends StatelessWidget {
  /// Creates a const [CheckboxMenuButton].
  ///
  /// The [child], [value], and [onChanged] attributes are required.
  const CheckboxMenuButton({
    super.key,
    required this.value,
    this.tristate = false,
    this.isError = false,
    required this.onChanged,
    this.onHover,
    this.onFocusChange,
    this.focusNode,
    this.shortcut,
    this.style,
    this.statesController,
    this.clipBehavior = Clip.none,
    this.trailingIcon,
    this.closeOnActivate = true,
    required this.child,
  });

  /// Whether this checkbox is checked.
  ///
  /// When [tristate] is true, a value of null corresponds to the mixed state.
  /// When [tristate] is false, this value must not be null.
  final bool? value;

  /// If true, then the checkbox's [value] can be true, false, or null.
  ///
  /// [CheckboxMenuButton] displays a dash when its value is null.
  ///
  /// When a tri-state checkbox ([tristate] is true) is tapped, its [onChanged]
  /// callback will be applied to true if the current value is false, to null if
  /// value is true, and to false if value is null (i.e. it cycles through false
  /// => true => null => false when tapped).
  ///
  /// If tristate is false (the default), [value] must not be null.
  final bool tristate;

  /// True if this checkbox wants to show an error state.
  ///
  /// The checkbox will have different default container color and check color when
  /// this is true. This is only used when [ThemeData.useMaterial3] is set to true.
  ///
  /// Defaults to false.
  final bool isError;

  /// Called when the value of the checkbox should change.
  ///
  /// The checkbox passes the new value to the callback but does not actually
  /// change state until the parent widget rebuilds the checkbox with the new
  /// value.
  ///
  /// If this callback is null, the menu item will be displayed as disabled
  /// and will not respond to input gestures.
  ///
  /// When the checkbox is tapped, if [tristate] is false (the default) then the
  /// [onChanged] callback will be applied to `!value`. If [tristate] is true
  /// this callback cycle from false to true to null and then back to false
  /// again.
  ///
  /// The callback provided to [onChanged] should update the state of the parent
  /// [StatefulWidget] using the [State.setState] method, so that the parent
  /// gets rebuilt; for example:
  ///
  /// ```dart
  /// CheckboxMenuButton(
  ///   value: _throwShotAway,
  ///   child: const Text('THROW'),
  ///   onChanged: (bool? newValue) {
  ///     setState(() {
  ///       _throwShotAway = newValue!;
  ///     });
  ///   },
  /// )
  /// ```
  final ValueChanged<bool?>? onChanged;

  /// Called when a pointer enters or exits the button response area.
  ///
  /// The value passed to the callback is true if a pointer has entered button
  /// area and false if a pointer has exited.
  final ValueChanged<bool>? onHover;

  /// Handler called when the focus changes.
  ///
  /// Called with true if this widget's node gains focus, and false if it loses
  /// focus.
  final ValueChanged<bool>? onFocusChange;

  /// {@macro flutter.widgets.Focus.focusNode}
  final FocusNode? focusNode;

  /// The optional shortcut that selects this [MenuItemButton].
  ///
  /// {@macro flutter.material.MenuBar.shortcuts_note}
  final MenuSerializableShortcut? shortcut;

  /// Customizes this button's appearance.
  ///
  /// Non-null properties of this style override the corresponding properties in
  /// [MenuItemButton.themeStyleOf] and [MenuItemButton.defaultStyleOf].
  /// [WidgetStateProperty]s that resolve to non-null values will similarly
  /// override the corresponding [WidgetStateProperty]s in
  /// [MenuItemButton.themeStyleOf] and [MenuItemButton.defaultStyleOf].
  ///
  /// Null by default.
  final ButtonStyle? style;

  /// {@macro flutter.material.inkwell.statesController}
  final MaterialStatesController? statesController;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.none].
  final Clip clipBehavior;

  /// An optional icon to display after the [child] label.
  final Widget? trailingIcon;

  /// {@macro flutter.material.menu_anchor.closeOnActivate}
  final bool closeOnActivate;

  /// The widget displayed in the center of this button.
  ///
  /// Typically this is the button's label, using a [Text] widget.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// Whether the button is enabled or disabled.
  ///
  /// To enable a button, set its [onChanged] property to a non-null value.
  bool get enabled => onChanged != null;

  @override
  Widget build(BuildContext context) {
    return MenuItemButton(
      key: key,
      onPressed:
          onChanged == null
              ? null
              : () {
                switch (value) {
                  case false:
                    onChanged!(true);
                  case true:
                    onChanged!(tristate ? null : false);
                  case null:
                    onChanged!(false);
                }
              },
      onHover: onHover,
      onFocusChange: onFocusChange,
      focusNode: focusNode,
      style: style,
      shortcut: shortcut,
      statesController: statesController,
      leadingIcon: ExcludeFocus(
        child: IgnorePointer(
          child: ConstrainedBox(
            constraints: const BoxConstraints(maxHeight: Checkbox.width, maxWidth: Checkbox.width),
            child: Checkbox(
              tristate: tristate,
              value: value,
              onChanged: onChanged,
              isError: isError,
            ),
          ),
        ),
      ),
      clipBehavior: clipBehavior,
      trailingIcon: trailingIcon,
      closeOnActivate: closeOnActivate,
      child: child,
    );
  }
}

/// A menu item that combines a [Radio] widget with a [MenuItemButton].
///
/// To style the radio button separately from the overall button, add a
/// [RadioTheme] ancestor.
///
/// {@tool dartpad}
/// This example shows a menu with three radio buttons with shortcuts that
/// changes the background color of the body when the buttons are selected.
///
/// ** See code in examples/api/lib/material/menu_anchor/radio_menu_button.0.dart **
/// {@end-tool}
///
/// See also:
///
/// - [MenuBar], a widget that creates a menu bar of cascading menu items.
/// - [MenuAnchor], a widget that defines a region which can host a cascading
///   menu.
class RadioMenuButton<T> extends StatelessWidget {
  /// Creates a const [RadioMenuButton].
  ///
  /// The [child] attribute is required.
  const RadioMenuButton({
    super.key,
    required this.value,
    required this.groupValue,
    required this.onChanged,
    this.toggleable = false,
    this.onHover,
    this.onFocusChange,
    this.focusNode,
    this.shortcut,
    this.style,
    this.statesController,
    this.clipBehavior = Clip.none,
    this.trailingIcon,
    this.closeOnActivate = true,
    required this.child,
  });

  /// The value represented by this radio button.
  ///
  /// This radio button is considered selected if its [value] matches the
  /// [groupValue].
  final T value;

  /// The currently selected value for a group of radio buttons.
  ///
  /// This radio button is considered selected if its [value] matches the
  /// [groupValue].
  final T? groupValue;

  /// Set to true if this radio button is allowed to be returned to an
  /// indeterminate state by selecting it again when selected.
  ///
  /// To indicate returning to an indeterminate state, [onChanged] will be
  /// called with null.
  ///
  /// If true, [onChanged] can be called with [value] when selected while
  /// [groupValue] != [value], or with null when selected again while
  /// [groupValue] == [value].
  ///
  /// If false, [onChanged] will be called with [value] when it is selected
  /// while [groupValue] != [value], and only by selecting another radio button
  /// in the group (i.e. changing the value of [groupValue]) can this radio
  /// button be unselected.
  ///
  /// The default is false.
  final bool toggleable;

  /// Called when the user selects this radio button.
  ///
  /// The radio button passes [value] as a parameter to this callback. The radio
  /// button does not actually change state until the parent widget rebuilds the
  /// radio button with the new [groupValue].
  ///
  /// If null, the radio button will be displayed as disabled.
  ///
  /// The provided callback will not be invoked if this radio button is already
  /// selected.
  ///
  /// The callback provided to [onChanged] should update the state of the parent
  /// [StatefulWidget] using the [State.setState] method, so that the parent
  /// gets rebuilt; for example:
  ///
  /// ```dart
  /// RadioMenuButton<SingingCharacter>(
  ///   value: SingingCharacter.lafayette,
  ///   groupValue: _character,
  ///   onChanged: (SingingCharacter? newValue) {
  ///     setState(() {
  ///       _character = newValue;
  ///     });
  ///   },
  ///   child: const Text('Lafayette'),
  /// )
  /// ```
  final ValueChanged<T?>? onChanged;

  /// Called when a pointer enters or exits the button response area.
  ///
  /// The value passed to the callback is true if a pointer has entered button
  /// area and false if a pointer has exited.
  final ValueChanged<bool>? onHover;

  /// Handler called when the focus changes.
  ///
  /// Called with true if this widget's node gains focus, and false if it loses
  /// focus.
  final ValueChanged<bool>? onFocusChange;

  /// {@macro flutter.widgets.Focus.focusNode}
  final FocusNode? focusNode;

  /// The optional shortcut that selects this [MenuItemButton].
  ///
  /// {@macro flutter.material.MenuBar.shortcuts_note}
  final MenuSerializableShortcut? shortcut;

  /// Customizes this button's appearance.
  ///
  /// Non-null properties of this style override the corresponding properties in
  /// [MenuItemButton.themeStyleOf] and [MenuItemButton.defaultStyleOf].
  /// [WidgetStateProperty]s that resolve to non-null values will similarly
  /// override the corresponding [WidgetStateProperty]s in
  /// [MenuItemButton.themeStyleOf] and [MenuItemButton.defaultStyleOf].
  ///
  /// Null by default.
  final ButtonStyle? style;

  /// {@macro flutter.material.inkwell.statesController}
  final MaterialStatesController? statesController;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.none].
  final Clip clipBehavior;

  /// An optional icon to display after the [child] label.
  final Widget? trailingIcon;

  /// {@macro flutter.material.menu_anchor.closeOnActivate}
  final bool closeOnActivate;

  /// The widget displayed in the center of this button.
  ///
  /// Typically this is the button's label, using a [Text] widget.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// Whether the button is enabled or disabled.
  ///
  /// To enable a button, set its [onChanged] property to a non-null value.
  bool get enabled => onChanged != null;

  @override
  Widget build(BuildContext context) {
    return MenuItemButton(
      key: key,
      onPressed:
          onChanged == null
              ? null
              : () {
                if (toggleable && groupValue == value) {
                  return onChanged!(null);
                }
                onChanged!(value);
              },
      onHover: onHover,
      onFocusChange: onFocusChange,
      focusNode: focusNode,
      style: style,
      shortcut: shortcut,
      statesController: statesController,
      leadingIcon: ExcludeFocus(
        child: IgnorePointer(
          child: ConstrainedBox(
            constraints: const BoxConstraints(maxHeight: Checkbox.width, maxWidth: Checkbox.width),
            child: Radio<T>(
              value: value,
              groupValue: groupValue,
              onChanged: onChanged,
              toggleable: toggleable,
            ),
          ),
        ),
      ),
      clipBehavior: clipBehavior,
      trailingIcon: trailingIcon,
      closeOnActivate: closeOnActivate,
      child: child,
    );
  }
}

/// A menu button that displays a cascading menu.
///
/// It can be used as part of a [MenuBar], or as a standalone widget.
///
/// This widget represents a menu item that has a submenu. Like the leaf
/// [MenuItemButton], it shows a label with an optional leading or trailing
/// icon, but additionally shows an arrow icon showing that it has a submenu.
///
/// By default the submenu will appear to the side of the controlling button.
/// The alignment and offset of the submenu can be controlled by setting
/// [MenuStyle.alignment] on the [style] and the [alignmentOffset] argument,
/// respectively.
///
/// When activated (by being clicked, through keyboard navigation, or via
/// hovering with a mouse), it will open a submenu containing the
/// [menuChildren].
///
/// If [menuChildren] is empty, then this menu item will appear disabled.
///
/// See also:
///
/// * [MenuItemButton], a widget that represents a leaf menu item that does not
///   host a submenu.
/// * [MenuBar], a widget that renders menu items in a row in a Material Design
///   style.
/// * [MenuAnchor], a widget that creates a region with a submenu and shows it
///   when requested.
/// * [PlatformMenuBar], a widget that renders similar menu bar items from a
///   [PlatformMenuItem] using platform-native APIs instead of Flutter.
class SubmenuButton extends StatefulWidget {
  /// Creates a const [SubmenuButton].
  ///
  /// The [child] and [menuChildren] attributes are required.
  const SubmenuButton({
    super.key,
    this.onHover,
    this.onFocusChange,
    this.onOpen,
    this.onClose,
    this.controller,
    this.style,
    this.menuStyle,
    this.alignmentOffset,
    this.clipBehavior = Clip.hardEdge,
    this.focusNode,
    this.statesController,
    this.leadingIcon,
    this.trailingIcon,
    this.submenuIcon,
    required this.menuChildren,
    required this.child,
  });

  /// Called when a pointer enters or exits the button response area.
  ///
  /// The value passed to the callback is true if a pointer has entered this
  /// part of the button and false if a pointer has exited.
  final ValueChanged<bool>? onHover;

  /// Handler called when the focus changes.
  ///
  /// Called with true if this widget's [focusNode] gains focus, and false if it
  /// loses focus.
  final ValueChanged<bool>? onFocusChange;

  /// A callback that is invoked when the menu is opened.
  final VoidCallback? onOpen;

  /// A callback that is invoked when the menu is closed.
  final VoidCallback? onClose;

  /// An optional [MenuController] for this submenu.
  final MenuController? controller;

  /// Customizes this button's appearance.
  ///
  /// Non-null properties of this style override the corresponding properties in
  /// [themeStyleOf] and [defaultStyleOf]. [WidgetStateProperty]s that resolve
  /// to non-null values will similarly override the corresponding
  /// [WidgetStateProperty]s in [themeStyleOf] and [defaultStyleOf].
  ///
  /// Null by default.
  final ButtonStyle? style;

  /// The [MenuStyle] of the menu specified by [menuChildren].
  ///
  /// Defaults to the value of [MenuThemeData.style] of the ambient [MenuTheme].
  final MenuStyle? menuStyle;

  /// The offset of the menu relative to the alignment origin determined by
  /// [MenuStyle.alignment] on the [style] attribute.
  ///
  /// Use this for fine adjustments of the menu placement.
  ///
  /// Defaults to an offset that takes into account the padding of the menu so
  /// that the top starting corner of the first menu item is aligned with the
  /// top of the [MenuAnchor] region.
  final Offset? alignmentOffset;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.hardEdge].
  final Clip clipBehavior;

  /// {@macro flutter.widgets.Focus.focusNode}
  final FocusNode? focusNode;

  /// {@macro flutter.material.inkwell.statesController}
  final MaterialStatesController? statesController;

  /// An optional icon to display before the [child].
  final Widget? leadingIcon;

  /// If provided, the widget replaces the default [SubmenuButton] arrow icon.
  ///
  /// Resolves in the following states:
  ///  * [WidgetState.disabled].
  ///  * [WidgetState.hovered].
  ///  * [WidgetState.focused].
  ///
  /// If this is null, then the value of [MenuThemeData.submenuIcon] is used.
  /// If that is also null, then defaults to a right arrow icon with the size
  /// of 24 pixels.
  final MaterialStateProperty<Widget?>? submenuIcon;

  /// An optional icon to display after the [child].
  final Widget? trailingIcon;

  /// The list of widgets that appear in the menu when it is opened.
  ///
  /// These can be any widget, but are typically either [MenuItemButton] or
  /// [SubmenuButton] widgets.
  ///
  /// If [menuChildren] is empty, then the button for this menu item will be
  /// disabled.
  final List<Widget> menuChildren;

  /// The widget displayed in the middle portion of this button.
  ///
  /// Typically this is the button's label, using a [Text] widget.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  @override
  State<SubmenuButton> createState() => _SubmenuButtonState();

  /// Defines the button's default appearance.
  ///
  /// {@macro flutter.material.text_button.default_style_of}
  ///
  /// {@macro flutter.material.text_button.material3_defaults}
  ButtonStyle defaultStyleOf(BuildContext context) {
    return _MenuButtonDefaultsM3(context);
  }

  /// Returns the [MenuButtonThemeData.style] of the closest [MenuButtonTheme]
  /// ancestor.
  ButtonStyle? themeStyleOf(BuildContext context) {
    return MenuButtonTheme.of(context).style;
  }

  /// A static convenience method that constructs a [SubmenuButton]'s
  /// [ButtonStyle] given simple values.
  ///
  /// The [foregroundColor] color is used to create a [WidgetStateProperty]
  /// [ButtonStyle.foregroundColor] value. Specify a value for [foregroundColor]
  /// to specify the color of the button's icons. Use [backgroundColor] for the
  /// button's background fill color. Use [disabledForegroundColor] and
  /// [disabledBackgroundColor] to specify the button's disabled icon and fill
  /// color.
  ///
  /// Similarly, the [enabledMouseCursor] and [disabledMouseCursor]
  /// parameters are used to construct [ButtonStyle.mouseCursor].
  ///
  /// The [iconColor], [disabledIconColor] are used to construct
  /// [ButtonStyle.iconColor] and [iconSize] is used to construct
  /// [ButtonStyle.iconSize].
  ///
  /// All of the other parameters are either used directly or used to create a
  /// [WidgetStateProperty] with a single value for all states.
  ///
  /// All parameters default to null, by default this method returns a
  /// [ButtonStyle] that doesn't override anything.
  ///
  /// For example, to override the default foreground color for a
  /// [SubmenuButton], as well as its overlay color, with all of the standard
  /// opacity adjustments for the pressed, focused, and hovered states, one
  /// could write:
  ///
  /// ```dart
  /// SubmenuButton(
  ///   leadingIcon: const Icon(Icons.pets),
  ///   style: SubmenuButton.styleFrom(foregroundColor: Colors.green),
  ///   menuChildren: const <Widget>[ /* ... */ ],
  ///   child: const Text('Button Label'),
  /// ),
  /// ```
  static ButtonStyle styleFrom({
    Color? foregroundColor,
    Color? backgroundColor,
    Color? disabledForegroundColor,
    Color? disabledBackgroundColor,
    Color? shadowColor,
    Color? surfaceTintColor,
    Color? iconColor,
    double? iconSize,
    Color? disabledIconColor,
    TextStyle? textStyle,
    Color? overlayColor,
    double? elevation,
    EdgeInsetsGeometry? padding,
    Size? minimumSize,
    Size? fixedSize,
    Size? maximumSize,
    MouseCursor? enabledMouseCursor,
    MouseCursor? disabledMouseCursor,
    BorderSide? side,
    OutlinedBorder? shape,
    VisualDensity? visualDensity,
    MaterialTapTargetSize? tapTargetSize,
    Duration? animationDuration,
    bool? enableFeedback,
    AlignmentGeometry? alignment,
    InteractiveInkFeatureFactory? splashFactory,
  }) {
    return TextButton.styleFrom(
      foregroundColor: foregroundColor,
      backgroundColor: backgroundColor,
      disabledBackgroundColor: disabledBackgroundColor,
      disabledForegroundColor: disabledForegroundColor,
      shadowColor: shadowColor,
      surfaceTintColor: surfaceTintColor,
      iconColor: iconColor,
      disabledIconColor: disabledIconColor,
      iconSize: iconSize,
      textStyle: textStyle,
      overlayColor: overlayColor,
      elevation: elevation,
      padding: padding,
      minimumSize: minimumSize,
      fixedSize: fixedSize,
      maximumSize: maximumSize,
      enabledMouseCursor: enabledMouseCursor,
      disabledMouseCursor: disabledMouseCursor,
      side: side,
      shape: shape,
      visualDensity: visualDensity,
      tapTargetSize: tapTargetSize,
      animationDuration: animationDuration,
      enableFeedback: enableFeedback,
      alignment: alignment,
      splashFactory: splashFactory,
    );
  }

  @override
  List<DiagnosticsNode> debugDescribeChildren() {
    return <DiagnosticsNode>[
      ...menuChildren.map<DiagnosticsNode>((Widget child) {
        return child.toDiagnosticsNode();
      }),
    ];
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<FocusNode?>('focusNode', focusNode));
    properties.add(DiagnosticsProperty<MenuStyle>('menuStyle', menuStyle, defaultValue: null));
    properties.add(DiagnosticsProperty<Offset>('alignmentOffset', alignmentOffset));
    properties.add(EnumProperty<Clip>('clipBehavior', clipBehavior));
  }
}

class _SubmenuButtonState extends State<SubmenuButton> {
  late final Map<Type, Action<Intent>> actions = <Type, Action<Intent>>{
    DirectionalFocusIntent: _SubmenuDirectionalFocusAction(submenu: this),
  };
  bool _waitingToFocusMenu = false;
  bool _isOpenOnFocusEnabled = true;
  MenuController? _internalMenuController;
  MenuController get _menuController => widget.controller ?? _internalMenuController!;
  _MenuAnchorState? get _parent => _MenuAnchorState._maybeOf(context);
  FocusNode? _internalFocusNode;
  FocusNode get _buttonFocusNode => widget.focusNode ?? _internalFocusNode!;
  bool get _enabled => widget.menuChildren.isNotEmpty;
  bool _isHovered = false;

  @override
  void initState() {
    super.initState();
    if (widget.focusNode == null) {
      _internalFocusNode = FocusNode();
      assert(() {
        _internalFocusNode?.debugLabel = '$SubmenuButton(${widget.child})';
        return true;
      }());
    }
    if (widget.controller == null) {
      _internalMenuController = MenuController();
    }
    _buttonFocusNode.addListener(_handleFocusChange);
  }

  @override
  void dispose() {
    _buttonFocusNode.removeListener(_handleFocusChange);
    _internalFocusNode?.dispose();
    _internalFocusNode = null;
    super.dispose();
  }

  @override
  void didUpdateWidget(SubmenuButton oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.focusNode != oldWidget.focusNode) {
      if (oldWidget.focusNode == null) {
        _internalFocusNode?.removeListener(_handleFocusChange);
        _internalFocusNode?.dispose();
        _internalFocusNode = null;
      } else {
        oldWidget.focusNode!.removeListener(_handleFocusChange);
      }
      if (widget.focusNode == null) {
        _internalFocusNode ??= FocusNode();
        assert(() {
          _internalFocusNode?.debugLabel = '$SubmenuButton(${widget.child})';
          return true;
        }());
      }
      _buttonFocusNode.addListener(_handleFocusChange);
    }
    if (widget.controller != oldWidget.controller) {
      _internalMenuController = (oldWidget.controller == null) ? null : MenuController();
    }
  }

  @override
  Widget build(BuildContext context) {
    Offset menuPaddingOffset = widget.alignmentOffset ?? Offset.zero;
    final EdgeInsets menuPadding = _computeMenuPadding(context);
    final Axis orientation = _parent?._orientation ?? Axis.vertical;
    // Move the submenu over by the size of the menu padding, so that
    // the first menu item aligns with the submenu button that opens it.
    menuPaddingOffset += switch ((orientation, Directionality.of(context))) {
      (Axis.horizontal, TextDirection.rtl) => Offset(menuPadding.right, 0),
      (Axis.horizontal, TextDirection.ltr) => Offset(-menuPadding.left, 0),
      (Axis.vertical, TextDirection.rtl) => Offset(0, -menuPadding.top),
      (Axis.vertical, TextDirection.ltr) => Offset(0, -menuPadding.top),
    };
    final Set<MaterialState> states = <MaterialState>{
      if (!_enabled) MaterialState.disabled,
      if (_isHovered) MaterialState.hovered,
      if (_buttonFocusNode.hasFocus) MaterialState.focused,
    };
    final Widget submenuIcon =
        widget.submenuIcon?.resolve(states) ??
        MenuTheme.of(context).submenuIcon?.resolve(states) ??
        const Icon(
          Icons.arrow_right, // Automatically switches with text direction.
          size: _kDefaultSubmenuIconSize,
        );

    return Actions(
      actions: actions,
      child: MenuAnchor(
        controller: _menuController,
        childFocusNode: _buttonFocusNode,
        alignmentOffset: menuPaddingOffset,
        clipBehavior: widget.clipBehavior,
        onClose: _onClose,
        onOpen: _onOpen,
        style: widget.menuStyle,
        builder: (BuildContext context, MenuController controller, Widget? child) {
          // Since we don't want to use the theme style or default style from the
          // TextButton, we merge the styles, merging them in the right order when
          // each type of style exists. Each "*StyleOf" function is only called
          // once.
          ButtonStyle mergedStyle =
              widget.themeStyleOf(context)?.merge(widget.defaultStyleOf(context)) ??
              widget.defaultStyleOf(context);
          mergedStyle = widget.style?.merge(mergedStyle) ?? mergedStyle;

          void toggleShowMenu() {
            if (controller._anchor == null) {
              return;
            }
            if (controller.isOpen) {
              controller.close();
            } else {
              controller.open();
            }
          }

          void handlePointerExit(PointerExitEvent event) {
            if (_isHovered) {
              widget.onHover?.call(false);
              _isHovered = false;
            }
          }

          // MouseRegion.onEnter and TextButton.onHover are called
          // if a button is hovered after scrolling. This interferes with
          // focus traversal and scroll position. MouseRegion.onHover avoids
          // this issue.
          void handlePointerHover(PointerHoverEvent event) {
            if (!_isHovered) {
              _isHovered = true;
              widget.onHover?.call(true);
              // Don't open the root menu bar menus on hover unless something else
              // is already open. This means that the user has to first click to
              // open a menu on the menu bar before hovering allows them to traverse
              // it.
              if (controller._anchor!._root._orientation == Axis.horizontal &&
                  !controller._anchor!._root._isOpen) {
                return;
              }

              controller.open();
              controller._anchor!._focusButton();
            }
          }

          child = MergeSemantics(
            child: Semantics(
              expanded: _enabled && controller.isOpen,
              child: TextButton(
                style: mergedStyle,
                focusNode: _buttonFocusNode,
                onFocusChange: _enabled ? widget.onFocusChange : null,
                onPressed: _enabled ? toggleShowMenu : null,
                isSemanticButton: null,
                child: _MenuItemLabel(
                  leadingIcon: widget.leadingIcon,
                  trailingIcon: widget.trailingIcon,
                  hasSubmenu: true,
                  showDecoration:
                      (controller._anchor!._parent?._orientation ?? Axis.horizontal) ==
                      Axis.vertical,
                  submenuIcon: submenuIcon,
                  child: child,
                ),
              ),
            ),
          );

          if (!_enabled) {
            return child;
          }

          child = MouseRegion(onHover: handlePointerHover, onExit: handlePointerExit, child: child);

          if (_platformSupportsAccelerators) {
            return MenuAcceleratorCallbackBinding(
              onInvoke: toggleShowMenu,
              hasSubmenu: true,
              child: child,
            );
          }

          return child;
        },
        menuChildren: widget.menuChildren,
        child: widget.child,
      ),
    );
  }

  void _onClose() {
    // After closing the children of this submenu, this submenu button will
    // regain focus. Because submenu buttons open on focus, this submenu will
    // immediately reopen. To prevent this from happening, we prevent focus on
    // SubmenuButtons that do not already have focus using the _openOnFocus
    // flag. This flag is reset after one frame.
    if (!_buttonFocusNode.hasFocus) {
      _isOpenOnFocusEnabled = false;
      SchedulerBinding.instance.addPostFrameCallback((Duration timestamp) {
        FocusManager.instance.applyFocusChangesIfNeeded();
        _isOpenOnFocusEnabled = true;
      }, debugLabel: 'MenuAnchor.preventOpenOnFocus');
    }
    widget.onClose?.call();
  }

  void _onOpen() {
    if (!_waitingToFocusMenu) {
      SchedulerBinding.instance.addPostFrameCallback((_) {
        _menuController._anchor?._focusButton();
        _waitingToFocusMenu = false;
      }, debugLabel: 'MenuAnchor.focus');
      _waitingToFocusMenu = true;
    }
    setState(() {
      /* Rebuild with updated controller.isOpen value */
    });
    widget.onOpen?.call();
  }

  EdgeInsets _computeMenuPadding(BuildContext context) {
    final MaterialStateProperty<EdgeInsetsGeometry?> insets =
        widget.menuStyle?.padding ??
        MenuTheme.of(context).style?.padding ??
        _MenuDefaultsM3(context).padding!;
    return insets
        .resolve(widget.statesController?.value ?? const <MaterialState>{})!
        .resolve(Directionality.of(context));
  }

  void _handleFocusChange() {
    if (_buttonFocusNode.hasPrimaryFocus) {
      if (!_menuController.isOpen && _isOpenOnFocusEnabled) {
        _menuController.open();
      }
    } else {
      if (!_menuController._anchor!._menuScopeNode.hasFocus && _menuController.isOpen) {
        _menuController.close();
      }
    }
  }
}

class _SubmenuDirectionalFocusAction extends DirectionalFocusAction {
  _SubmenuDirectionalFocusAction({required this.submenu});

  final _SubmenuButtonState submenu;

  _MenuAnchorState get _anchor => submenu._menuController._anchor!;
  FocusNode get _buttonFocusNode => submenu._buttonFocusNode;
  _MenuAnchorState? get _parent => _anchor._parent;
  bool get _isParentRoot => _parent?._isRoot ?? false;

  /// The orientation of the menu that contains this submenu button.
  Axis? get _orientation => _parent?._orientation;

  /// Whether the anchor that intercepted this DirectionalFocusAction is a submenu.
  bool get isSubmenu => submenu._buttonFocusNode.hasPrimaryFocus;

  @override
  void invoke(DirectionalFocusIntent intent) {
    assert(_debugMenuInfo('${intent.direction}: Invoking directional focus intent.'));
    final TextDirection directionality = Directionality.of(submenu.context);
    switch ((_orientation, directionality, intent.direction)) {
      case (Axis.horizontal, TextDirection.ltr, TraversalDirection.left):
      case (Axis.horizontal, TextDirection.rtl, TraversalDirection.right):
        assert(_debugMenuInfo('Moving to previous $MenuBar item'));
        // Focus this MenuBar SubmenuButton, then move focus to the previous focusable
        // MenuBar item.
        _buttonFocusNode
          ..requestFocus()
          ..previousFocus();
        return;
      case (Axis.horizontal, TextDirection.ltr, TraversalDirection.right):
      case (Axis.horizontal, TextDirection.rtl, TraversalDirection.left):
        assert(_debugMenuInfo('Moving to next $MenuBar item'));
        // Focus this MenuBar SubmenuButton, then move focus to the next focusable
        // MenuBar item.
        _buttonFocusNode
          ..requestFocus()
          ..nextFocus();
        return;
      case (Axis.horizontal, _, TraversalDirection.down):
        if (isSubmenu) {
          // If this is a top-level (horizontal) button in a menubar, focus the
          // first item in this button's submenu.
          final FocusNode? firstItem = _anchor._firstItemFocusNode;
          if (firstItem?.canRequestFocus ?? false) {
            firstItem!.requestFocus();
          }
          return;
        }
      case (Axis.horizontal, _, TraversalDirection.up):
        if (isSubmenu) {
          // If this is a top-level (horizontal) button in a menubar, focus the
          // last item in this button's submenu. This makes navigating into
          // upward-oriented submenus more intuitive.
          final FocusNode? lastItem = _anchor._lastItemFocusNode;
          if (lastItem?.canRequestFocus ?? false) {
            lastItem!.requestFocus();
          }
          return;
        }
      case (Axis.vertical, TextDirection.ltr, TraversalDirection.left):
      case (Axis.vertical, TextDirection.rtl, TraversalDirection.right):
        if (_parent?._parent?._orientation == Axis.horizontal) {
          if (isSubmenu) {
            _parent!.widget.childFocusNode
              ?..requestFocus()
              ..previousFocus();
          } else {
            assert(_debugMenuInfo('Exiting submenu'));
            // MenuBar SubmenuButton => SubmenuButton => child
            // Focus the parent SubmenuButton anchor attached to this child.
            _buttonFocusNode.requestFocus();
          }
        } else {
          if (isSubmenu) {
            if (_isParentRoot) {
              // Moving in the closing direction while focused on a
              // SubmenuButton within a root MenuAnchor menu should not close
              // the menu.
              return;
            }
            _parent
              ?.._focusButton()
              .._close();
          } else {
            // If focus is not on a submenu button, closing the anchor this item
            // presides in will close the menu and focus the anchor button.
            _anchor._close();
          }
          assert(_debugMenuInfo('Exiting submenu'));
        }
        return;
      case (Axis.vertical, TextDirection.ltr, TraversalDirection.right) when isSubmenu:
      case (Axis.vertical, TextDirection.rtl, TraversalDirection.left) when isSubmenu:
        assert(_debugMenuInfo('Entering submenu'));
        if (_anchor._isOpen) {
          _anchor._firstItemFocusNode?.requestFocus();
        } else {
          _anchor._open();
          SchedulerBinding.instance.addPostFrameCallback((Duration timestamp) {
            if (_anchor._isOpen) {
              _anchor._firstItemFocusNode?.requestFocus();
            }
          });
        }
        return;
      default:
        break;
    }

    Actions.maybeInvoke(submenu.context, intent);
  }
}

/// An action that closes all the menus associated with the given
/// [MenuController].
///
/// See also:
///
///  * [MenuAnchor], a widget that hosts a cascading submenu.
///  * [MenuBar], a widget that defines a menu bar with cascading submenus.
class DismissMenuAction extends DismissAction {
  /// Creates a [DismissMenuAction].
  DismissMenuAction({required this.controller});

  /// The [MenuController] associated with the menus that should be closed.
  final MenuController controller;

  @override
  void invoke(DismissIntent intent) {
    assert(_debugMenuInfo('$runtimeType: Dismissing all open menus.'));
    controller._anchor!._root._close();
  }

  @override
  bool isEnabled(DismissIntent intent) {
    return controller.isOpen;
  }
}

/// A helper class used to generate shortcut labels for a
/// [MenuSerializableShortcut] (a subset of the subclasses of
/// [ShortcutActivator]).
///
/// This helper class is typically used by the [MenuItemButton] and
/// [SubmenuButton] classes to display a label for their assigned shortcuts.
///
/// Call [getShortcutLabel] with the [MenuSerializableShortcut] to get a label
/// for it.
///
/// For instance, calling [getShortcutLabel] with `SingleActivator(trigger:
/// LogicalKeyboardKey.keyA, control: true)` would return "⌃ A" on macOS, "Ctrl
/// A" in an US English locale, and "Strg A" in a German locale.
class _LocalizedShortcutLabeler {
  _LocalizedShortcutLabeler._();

  static _LocalizedShortcutLabeler? _instance;

  static final Map<LogicalKeyboardKey, String> _shortcutGraphicEquivalents =
      <LogicalKeyboardKey, String>{
        LogicalKeyboardKey.arrowLeft: '←',
        LogicalKeyboardKey.arrowRight: '→',
        LogicalKeyboardKey.arrowUp: '↑',
        LogicalKeyboardKey.arrowDown: '↓',
        LogicalKeyboardKey.enter: '↵',
      };

  static final Set<LogicalKeyboardKey> _modifiers = <LogicalKeyboardKey>{
    LogicalKeyboardKey.alt,
    LogicalKeyboardKey.control,
    LogicalKeyboardKey.meta,
    LogicalKeyboardKey.shift,
    LogicalKeyboardKey.altLeft,
    LogicalKeyboardKey.controlLeft,
    LogicalKeyboardKey.metaLeft,
    LogicalKeyboardKey.shiftLeft,
    LogicalKeyboardKey.altRight,
    LogicalKeyboardKey.controlRight,
    LogicalKeyboardKey.metaRight,
    LogicalKeyboardKey.shiftRight,
  };

  /// Return the instance for this singleton.
  static _LocalizedShortcutLabeler get instance {
    return _instance ??= _LocalizedShortcutLabeler._();
  }

  // Caches the created shortcut key maps so that creating one of these isn't
  // expensive after the first time for each unique localizations object.
  final Map<MaterialLocalizations, Map<LogicalKeyboardKey, String>> _cachedShortcutKeys =
      <MaterialLocalizations, Map<LogicalKeyboardKey, String>>{};

  /// Returns the label to be shown to the user in the UI when a
  /// [MenuSerializableShortcut] is used as a keyboard shortcut.
  ///
  /// When [defaultTargetPlatform] is [TargetPlatform.macOS] or
  /// [TargetPlatform.iOS], this will return graphical key representations when
  /// it can. For instance, the default [LogicalKeyboardKey.shift] will return
  /// '⇧', and the arrow keys will return arrows. The key
  /// [LogicalKeyboardKey.meta] will show as '⌘', [LogicalKeyboardKey.control]
  /// will show as '˄', and [LogicalKeyboardKey.alt] will show as '⌥'.
  ///
  /// The keys are joined by spaces on macOS and iOS, and by "+" on other
  /// platforms.
  String getShortcutLabel(MenuSerializableShortcut shortcut, MaterialLocalizations localizations) {
    final ShortcutSerialization serialized = shortcut.serializeForMenu();
    final String keySeparator;
    if (_usesSymbolicModifiers) {
      // Use "⌃ ⇧ A" style on macOS and iOS.
      keySeparator = ' ';
    } else {
      // Use "Ctrl+Shift+A" style.
      keySeparator = '+';
    }
    if (serialized.trigger != null) {
      final LogicalKeyboardKey trigger = serialized.trigger!;
      final List<String> modifiers = <String>[
        if (_usesSymbolicModifiers) ...<String>[
          // macOS/iOS platform convention uses this ordering, with ⌘ always last.
          if (serialized.control!) _getModifierLabel(LogicalKeyboardKey.control, localizations),
          if (serialized.alt!) _getModifierLabel(LogicalKeyboardKey.alt, localizations),
          if (serialized.shift!) _getModifierLabel(LogicalKeyboardKey.shift, localizations),
          if (serialized.meta!) _getModifierLabel(LogicalKeyboardKey.meta, localizations),
        ] else ...<String>[
          // This order matches the LogicalKeySet version.
          if (serialized.alt!) _getModifierLabel(LogicalKeyboardKey.alt, localizations),
          if (serialized.control!) _getModifierLabel(LogicalKeyboardKey.control, localizations),
          if (serialized.meta!) _getModifierLabel(LogicalKeyboardKey.meta, localizations),
          if (serialized.shift!) _getModifierLabel(LogicalKeyboardKey.shift, localizations),
        ],
      ];
      String? shortcutTrigger;
      final int logicalKeyId = trigger.keyId;
      if (_shortcutGraphicEquivalents.containsKey(trigger)) {
        shortcutTrigger = _shortcutGraphicEquivalents[trigger];
      } else {
        // Otherwise, look it up, and if we don't have a translation for it,
        // then fall back to the key label.
        shortcutTrigger = _getLocalizedName(trigger, localizations);
        if (shortcutTrigger == null && logicalKeyId & LogicalKeyboardKey.planeMask == 0x0) {
          // If the trigger is a Unicode-character-producing key, then use the
          // character.
          shortcutTrigger =
              String.fromCharCode(logicalKeyId & LogicalKeyboardKey.valueMask).toUpperCase();
        }
        // Fall back to the key label if all else fails.
        shortcutTrigger ??= trigger.keyLabel;
      }
      return <String>[
        ...modifiers,
        if (shortcutTrigger != null && shortcutTrigger.isNotEmpty) shortcutTrigger,
      ].join(keySeparator);
    } else if (serialized.character != null) {
      final List<String> modifiers = <String>[
        // Character based shortcuts cannot check shifted keys.
        if (_usesSymbolicModifiers) ...<String>[
          // macOS/iOS platform convention uses this ordering, with ⌘ always last.
          if (serialized.control!) _getModifierLabel(LogicalKeyboardKey.control, localizations),
          if (serialized.alt!) _getModifierLabel(LogicalKeyboardKey.alt, localizations),
          if (serialized.meta!) _getModifierLabel(LogicalKeyboardKey.meta, localizations),
        ] else ...<String>[
          // This order matches the LogicalKeySet version.
          if (serialized.alt!) _getModifierLabel(LogicalKeyboardKey.alt, localizations),
          if (serialized.control!) _getModifierLabel(LogicalKeyboardKey.control, localizations),
          if (serialized.meta!) _getModifierLabel(LogicalKeyboardKey.meta, localizations),
        ],
      ];
      return <String>[...modifiers, serialized.character!].join(keySeparator);
    }
    throw UnimplementedError(
      'Shortcut labels for ShortcutActivators that do not implement '
      'MenuSerializableShortcut (e.g. ShortcutActivators other than SingleActivator or '
      'CharacterActivator) are not supported.',
    );
  }

  // Tries to look up the key in an internal table, and if it can't find it,
  // then fall back to the key's keyLabel.
  String? _getLocalizedName(LogicalKeyboardKey key, MaterialLocalizations localizations) {
    // Since this is an expensive table to build, we cache it based on the
    // localization object. There's currently no way to clear the cache, but
    // it's unlikely that more than one or two will be cached for each run, and
    // they're not huge.
    _cachedShortcutKeys[localizations] ??= <LogicalKeyboardKey, String>{
      LogicalKeyboardKey.altGraph: localizations.keyboardKeyAltGraph,
      LogicalKeyboardKey.backspace: localizations.keyboardKeyBackspace,
      LogicalKeyboardKey.capsLock: localizations.keyboardKeyCapsLock,
      LogicalKeyboardKey.channelDown: localizations.keyboardKeyChannelDown,
      LogicalKeyboardKey.channelUp: localizations.keyboardKeyChannelUp,
      LogicalKeyboardKey.delete: localizations.keyboardKeyDelete,
      LogicalKeyboardKey.eject: localizations.keyboardKeyEject,
      LogicalKeyboardKey.end: localizations.keyboardKeyEnd,
      LogicalKeyboardKey.escape: localizations.keyboardKeyEscape,
      LogicalKeyboardKey.fn: localizations.keyboardKeyFn,
      LogicalKeyboardKey.home: localizations.keyboardKeyHome,
      LogicalKeyboardKey.insert: localizations.keyboardKeyInsert,
      LogicalKeyboardKey.numLock: localizations.keyboardKeyNumLock,
      LogicalKeyboardKey.numpad1: localizations.keyboardKeyNumpad1,
      LogicalKeyboardKey.numpad2: localizations.keyboardKeyNumpad2,
      LogicalKeyboardKey.numpad3: localizations.keyboardKeyNumpad3,
      LogicalKeyboardKey.numpad4: localizations.keyboardKeyNumpad4,
      LogicalKeyboardKey.numpad5: localizations.keyboardKeyNumpad5,
      LogicalKeyboardKey.numpad6: localizations.keyboardKeyNumpad6,
      LogicalKeyboardKey.numpad7: localizations.keyboardKeyNumpad7,
      LogicalKeyboardKey.numpad8: localizations.keyboardKeyNumpad8,
      LogicalKeyboardKey.numpad9: localizations.keyboardKeyNumpad9,
      LogicalKeyboardKey.numpad0: localizations.keyboardKeyNumpad0,
      LogicalKeyboardKey.numpadAdd: localizations.keyboardKeyNumpadAdd,
      LogicalKeyboardKey.numpadComma: localizations.keyboardKeyNumpadComma,
      LogicalKeyboardKey.numpadDecimal: localizations.keyboardKeyNumpadDecimal,
      LogicalKeyboardKey.numpadDivide: localizations.keyboardKeyNumpadDivide,
      LogicalKeyboardKey.numpadEnter: localizations.keyboardKeyNumpadEnter,
      LogicalKeyboardKey.numpadEqual: localizations.keyboardKeyNumpadEqual,
      LogicalKeyboardKey.numpadMultiply: localizations.keyboardKeyNumpadMultiply,
      LogicalKeyboardKey.numpadParenLeft: localizations.keyboardKeyNumpadParenLeft,
      LogicalKeyboardKey.numpadParenRight: localizations.keyboardKeyNumpadParenRight,
      LogicalKeyboardKey.numpadSubtract: localizations.keyboardKeyNumpadSubtract,
      LogicalKeyboardKey.pageDown: localizations.keyboardKeyPageDown,
      LogicalKeyboardKey.pageUp: localizations.keyboardKeyPageUp,
      LogicalKeyboardKey.power: localizations.keyboardKeyPower,
      LogicalKeyboardKey.powerOff: localizations.keyboardKeyPowerOff,
      LogicalKeyboardKey.printScreen: localizations.keyboardKeyPrintScreen,
      LogicalKeyboardKey.scrollLock: localizations.keyboardKeyScrollLock,
      LogicalKeyboardKey.select: localizations.keyboardKeySelect,
      LogicalKeyboardKey.space: localizations.keyboardKeySpace,
    };
    return _cachedShortcutKeys[localizations]![key];
  }

  String _getModifierLabel(LogicalKeyboardKey modifier, MaterialLocalizations localizations) {
    assert(_modifiers.contains(modifier), '${modifier.keyLabel} is not a modifier key');
    if (modifier == LogicalKeyboardKey.meta ||
        modifier == LogicalKeyboardKey.metaLeft ||
        modifier == LogicalKeyboardKey.metaRight) {
      switch (defaultTargetPlatform) {
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
          return localizations.keyboardKeyMeta;
        case TargetPlatform.windows:
          return localizations.keyboardKeyMetaWindows;
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          return '⌘';
      }
    }
    if (modifier == LogicalKeyboardKey.alt ||
        modifier == LogicalKeyboardKey.altLeft ||
        modifier == LogicalKeyboardKey.altRight) {
      switch (defaultTargetPlatform) {
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          return localizations.keyboardKeyAlt;
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          return '⌥';
      }
    }
    if (modifier == LogicalKeyboardKey.control ||
        modifier == LogicalKeyboardKey.controlLeft ||
        modifier == LogicalKeyboardKey.controlRight) {
      // '⎈' (a boat helm wheel, not an asterisk) is apparently the standard
      // icon for "control", but only seems to appear on the French Canadian
      // keyboard. A '✲' (an open center asterisk) appears on some Microsoft
      // keyboards. For all but macOS (which has standardized on "⌃", it seems),
      // we just return the local translation of "Ctrl".
      switch (defaultTargetPlatform) {
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          return localizations.keyboardKeyControl;
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          return '⌃';
      }
    }
    if (modifier == LogicalKeyboardKey.shift ||
        modifier == LogicalKeyboardKey.shiftLeft ||
        modifier == LogicalKeyboardKey.shiftRight) {
      switch (defaultTargetPlatform) {
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          return localizations.keyboardKeyShift;
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          return '⇧';
      }
    }
    throw ArgumentError('Keyboard key ${modifier.keyLabel} is not a modifier.');
  }
}

class _MenuAnchorScope extends InheritedWidget {
  const _MenuAnchorScope({
    required super.child,
    required this.anchorKey,
    required this.anchor,
    required this.isOpen,
  });

  final GlobalKey anchorKey;
  final _MenuAnchorState anchor;
  final bool isOpen;

  @override
  bool updateShouldNotify(_MenuAnchorScope oldWidget) {
    return anchorKey != oldWidget.anchorKey ||
        anchor != oldWidget.anchor ||
        isOpen != oldWidget.isOpen;
  }
}

/// MenuBar-specific private specialization of [MenuAnchor] so that it can act
/// differently in regards to orientation, how open works, and what gets built.
class _MenuBarAnchor extends MenuAnchor {
  const _MenuBarAnchor({
    required super.menuChildren,
    super.controller,
    super.clipBehavior,
    super.style,
  });

  @override
  State<MenuAnchor> createState() => _MenuBarAnchorState();
}

class _MenuBarAnchorState extends _MenuAnchorState {
  late final Map<Type, Action<Intent>> actions = <Type, Action<Intent>>{
    DismissIntent: DismissMenuAction(controller: _menuController),
  };

  @override
  bool get _isOpen {
    // If it's a bar, then it's "open" if any of its children are open.
    for (final _MenuAnchorState child in _anchorChildren) {
      if (child._isOpen) {
        return true;
      }
    }
    return false;
  }

  @override
  Axis get _orientation => Axis.horizontal;

  @override
  Widget _buildContents(BuildContext context) {
    final bool isOpen = _isOpen;
    return FocusScope(
      node: _menuScopeNode,
      skipTraversal: !isOpen,
      canRequestFocus: isOpen,
      descendantsAreFocusable: true,
      child: ExcludeFocus(
        excluding: !isOpen,
        child: Shortcuts(
          shortcuts: _kMenuTraversalShortcuts,
          child: Actions(
            actions: actions,
            child: _MenuPanel(
              menuStyle: widget.style,
              clipBehavior: widget.clipBehavior,
              orientation: Axis.horizontal,
              children: widget.menuChildren,
            ),
          ),
        ),
      ),
    );
  }

  @override
  void _open({Offset? position}) {
    assert(_menuController._anchor == this);
    // Menu bars can't be opened, because they're already always open.
    return;
  }
}

/// An [InheritedWidget] that provides a descendant [MenuAcceleratorLabel] with
/// the function to invoke when the accelerator is pressed.
///
/// This is used when creating your own custom menu item for use with
/// [MenuAnchor] or [MenuBar]. Provided menu items such as [MenuItemButton] and
/// [SubmenuButton] already supply this wrapper internally.
class MenuAcceleratorCallbackBinding extends InheritedWidget {
  /// Create a const [MenuAcceleratorCallbackBinding].
  ///
  /// The [child] parameter is required.
  const MenuAcceleratorCallbackBinding({
    super.key,
    this.onInvoke,
    this.hasSubmenu = false,
    required super.child,
  });

  /// The function that pressing the accelerator defined in a descendant
  /// [MenuAcceleratorLabel] will invoke.
  ///
  /// If set to null, then the accelerator won't be enabled.
  final VoidCallback? onInvoke;

  /// Whether or not the associated label will host its own submenu or not.
  ///
  /// This setting determines when accelerators are active, since accelerators
  /// for menu items that open submenus shouldn't be active when the submenu is
  /// open.
  final bool hasSubmenu;

  @override
  bool updateShouldNotify(MenuAcceleratorCallbackBinding oldWidget) {
    return onInvoke != oldWidget.onInvoke || hasSubmenu != oldWidget.hasSubmenu;
  }

  /// Returns the active [MenuAcceleratorCallbackBinding] in the given context, if any,
  /// and creates a dependency relationship that will rebuild the context when
  /// [onInvoke] changes.
  ///
  /// If no [MenuAcceleratorCallbackBinding] is found, returns null.
  ///
  /// See also:
  ///
  /// * [of], which is similar, but asserts if no [MenuAcceleratorCallbackBinding]
  ///   is found.
  static MenuAcceleratorCallbackBinding? maybeOf(BuildContext context) {
    return context.dependOnInheritedWidgetOfExactType<MenuAcceleratorCallbackBinding>();
  }

  /// Returns the active [MenuAcceleratorCallbackBinding] in the given context, and
  /// creates a dependency relationship that will rebuild the context when
  /// [onInvoke] changes.
  ///
  /// If no [MenuAcceleratorCallbackBinding] is found, returns will assert in debug mode
  /// and throw an exception in release mode.
  ///
  /// See also:
  ///
  /// * [maybeOf], which is similar, but returns null if no
  ///   [MenuAcceleratorCallbackBinding] is found.
  static MenuAcceleratorCallbackBinding of(BuildContext context) {
    final MenuAcceleratorCallbackBinding? result = maybeOf(context);
    assert(() {
      if (result == null) {
        throw FlutterError(
          'MenuAcceleratorWrapper.of() was called with a context that does not '
          'contain a MenuAcceleratorWrapper in the given context.\n'
          'No MenuAcceleratorWrapper ancestor could be found in the context that '
          'was passed to MenuAcceleratorWrapper.of(). This can happen because '
          'you are using a widget that looks for a MenuAcceleratorWrapper '
          'ancestor, and do not have a MenuAcceleratorWrapper widget ancestor.\n'
          'The context used was:\n'
          '  $context',
        );
      }
      return true;
    }());
    return result!;
  }
}

/// The type of builder function used for building a [MenuAcceleratorLabel]'s
/// [MenuAcceleratorLabel.builder] function.
///
/// {@template flutter.material.menu_anchor.menu_accelerator_child_builder.args}
/// The arguments to the function are as follows:
///
/// * The `context` supplies the [BuildContext] to use.
/// * The `label` is the [MenuAcceleratorLabel.label] attribute for the relevant
///   [MenuAcceleratorLabel] with the accelerator markers stripped out of it.
/// * The `index` is the index of the accelerator character within the
///   `label.characters` that applies to this accelerator. If it is -1, then the
///   accelerator should not be highlighted. Otherwise, the given character
///   should be highlighted somehow in the rendered label (typically with an
///   underscore). Importantly, `index` is not an index into the [String]
///   `label`, it is an index into the [Characters] iterable returned by
///   `label.characters`, so that it is in terms of user-visible characters
///   (a.k.a. grapheme clusters), not Unicode code points.
/// {@endtemplate}
///
/// See also:
///
/// * [MenuAcceleratorLabel.defaultLabelBuilder], which is the implementation
///   used as the default value for [MenuAcceleratorLabel.builder].
typedef MenuAcceleratorChildBuilder =
    Widget Function(BuildContext context, String label, int index);

/// A widget that draws the label text for a menu item (typically a
/// [MenuItemButton] or [SubmenuButton]) and renders its child with information
/// about the currently active keyboard accelerator.
///
/// On platforms other than macOS and iOS, this widget listens for the Alt key
/// to be pressed, and when it is down, will update the label by calling the
/// builder again with the position of the accelerator in the label string.
/// While the Alt key is pressed, it registers a shortcut with the
/// [ShortcutRegistry] mapped to a [VoidCallbackIntent] containing the callback
/// defined by the nearest [MenuAcceleratorCallbackBinding].
///
/// Because the accelerators are registered with the [ShortcutRegistry], any
/// other shortcuts in the widget tree between the [primaryFocus] and the
/// [ShortcutRegistry] that define Alt-based shortcuts using the same keys will
/// take precedence over the accelerators.
///
/// Because accelerators aren't used on macOS and iOS, the label ignores the Alt
/// key on those platforms, and the [builder] is always given -1 as an
/// accelerator index. Accelerator labels are still stripped of their
/// accelerator markers.
///
/// The built-in menu items [MenuItemButton] and [SubmenuButton] already provide
/// the appropriate [MenuAcceleratorCallbackBinding], so unless you are creating
/// your own custom menu item type that takes a [MenuAcceleratorLabel], it is
/// not necessary to provide one.
///
/// {@template flutter.material.MenuAcceleratorLabel.accelerator_sample}
/// {@tool dartpad} This example shows a [MenuBar] that handles keyboard
/// accelerators using [MenuAcceleratorLabel]. To use the accelerators, press
/// the Alt key to see which letters are underlined in the menu bar, and then
/// press the appropriate letter. Accelerators are not supported on macOS or iOS
/// since those platforms don't support them natively, so this demo will only
/// show a regular Material menu bar on those platforms.
///
/// ** See code in examples/api/lib/material/menu_anchor/menu_accelerator_label.0.dart **
/// {@end-tool}
/// {@endtemplate}
class MenuAcceleratorLabel extends StatefulWidget {
  /// Creates a const [MenuAcceleratorLabel].
  ///
  /// The [label] parameter is required.
  const MenuAcceleratorLabel(this.label, {super.key, this.builder = defaultLabelBuilder});

  /// The label string that should be displayed.
  ///
  /// The label string provides the label text, as well as the possible
  /// characters which could be used as accelerators in the menu system.
  ///
  /// {@template flutter.material.menu_anchor.menu_accelerator_label.label}
  /// To indicate which letters in the label are to be used as accelerators, add
  /// an "&" character before the character in the string. If more than one
  /// character has an "&" in front of it, then the characters appearing earlier
  /// in the string are preferred. To represent a literal "&", insert "&&" into
  /// the string. All other ampersands will be removed from the string before
  /// calling [MenuAcceleratorLabel.builder]. Bare ampersands at the end of the
  /// string or before whitespace are stripped and ignored.
  /// {@endtemplate}
  ///
  /// See also:
  ///
  /// * [displayLabel], which returns the [label] with all of the ampersands
  ///   stripped out of it, and double ampersands converted to ampersands.
  /// * [stripAcceleratorMarkers], which returns the supplied string with all of
  ///   the ampersands stripped out of it, and double ampersands converted to
  ///   ampersands, and optionally calls a callback with the index of the
  ///   accelerator character found.
  final String label;

  /// Returns the [label] with any accelerator markers removed.
  ///
  /// This getter just calls [stripAcceleratorMarkers] with the [label].
  String get displayLabel => stripAcceleratorMarkers(label);

  /// The optional [MenuAcceleratorChildBuilder] which is used to build the
  /// widget that displays the label itself.
  ///
  /// The [defaultLabelBuilder] function serves as the default value for
  /// [builder], rendering the label as a [RichText] widget with appropriate
  /// [TextSpan]s for rendering the label with an underscore under the selected
  /// accelerator for the label when accelerators have been activated.
  ///
  /// {@macro flutter.material.menu_anchor.menu_accelerator_child_builder.args}
  ///
  /// When writing the builder function, it's not necessary to take the current
  /// platform into account. On platforms which don't support accelerators (e.g.
  /// macOS and iOS), the passed accelerator index will always be -1, and the
  /// accelerator markers will already be stripped.
  final MenuAcceleratorChildBuilder builder;

  /// Whether [label] contains an accelerator definition.
  ///
  /// {@macro flutter.material.menu_anchor.menu_accelerator_label.label}
  bool get hasAccelerator => RegExp(r'&(?!([&\s]|$))').hasMatch(label);

  /// Serves as the default value for [builder], rendering the label as a
  /// [RichText] widget with appropriate [TextSpan]s for rendering the label
  /// with an underscore under the selected accelerator for the label when the
  /// [index] is non-negative, and a [Text] widget when the [index] is negative.
  ///
  /// {@macro flutter.material.menu_anchor.menu_accelerator_child_builder.args}
  static Widget defaultLabelBuilder(BuildContext context, String label, int index) {
    if (index < 0) {
      return Text(label);
    }
    final TextStyle defaultStyle = DefaultTextStyle.of(context).style;
    final Characters characters = label.characters;
    return RichText(
      text: TextSpan(
        children: <TextSpan>[
          if (index > 0)
            TextSpan(text: characters.getRange(0, index).toString(), style: defaultStyle),
          TextSpan(
            text: characters.getRange(index, index + 1).toString(),
            style: defaultStyle.copyWith(decoration: TextDecoration.underline),
          ),
          if (index < characters.length - 1)
            TextSpan(text: characters.getRange(index + 1).toString(), style: defaultStyle),
        ],
      ),
    );
  }

  /// Strips out any accelerator markers from the given [label], and unescapes
  /// any escaped ampersands.
  ///
  /// If [setIndex] is supplied, it will be called before this function returns
  /// with the index in the returned string of the accelerator character.
  ///
  /// {@macro flutter.material.menu_anchor.menu_accelerator_label.label}
  static String stripAcceleratorMarkers(String label, {void Function(int index)? setIndex}) {
    int quotedAmpersands = 0;
    final StringBuffer displayLabel = StringBuffer();
    int acceleratorIndex = -1;
    // Use characters so that we don't split up surrogate pairs and interpret
    // them incorrectly.
    final Characters labelChars = label.characters;
    final Characters ampersand = '&'.characters;
    bool lastWasAmpersand = false;
    for (int i = 0; i < labelChars.length; i += 1) {
      // Stop looking one before the end, since a single ampersand at the end is
      // just treated as a quoted ampersand.
      final Characters character = labelChars.characterAt(i);
      if (lastWasAmpersand) {
        lastWasAmpersand = false;
        displayLabel.write(character);
        continue;
      }
      if (character != ampersand) {
        displayLabel.write(character);
        continue;
      }
      if (i == labelChars.length - 1) {
        // Strip bare ampersands at the end of a string.
        break;
      }
      lastWasAmpersand = true;
      final Characters acceleratorCharacter = labelChars.characterAt(i + 1);
      if (acceleratorIndex == -1 &&
          acceleratorCharacter != ampersand &&
          acceleratorCharacter.toString().trim().isNotEmpty) {
        // Don't set the accelerator index if the character is an ampersand,
        // or whitespace.
        acceleratorIndex = i - quotedAmpersands;
      }
      // As we encounter '&<character>' pairs, the following indices must be
      // adjusted so that they correspond with indices in the stripped string.
      quotedAmpersands += 1;
    }
    setIndex?.call(acceleratorIndex);
    return displayLabel.toString();
  }

  @override
  State<MenuAcceleratorLabel> createState() => _MenuAcceleratorLabelState();

  @override
  String toString({DiagnosticLevel minLevel = DiagnosticLevel.info}) {
    return '$MenuAcceleratorLabel("$label")';
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(StringProperty('label', label));
  }
}

class _MenuAcceleratorLabelState extends State<MenuAcceleratorLabel> {
  late String _displayLabel;
  int _acceleratorIndex = -1;
  MenuAcceleratorCallbackBinding? _binding;
  _MenuAnchorState? _anchor;
  ShortcutRegistry? _shortcutRegistry;
  ShortcutRegistryEntry? _shortcutRegistryEntry;
  bool _showAccelerators = false;

  @override
  void initState() {
    super.initState();
    if (_platformSupportsAccelerators) {
      _showAccelerators = _altIsPressed();
      HardwareKeyboard.instance.addHandler(_listenToKeyEvent);
    }
    _updateDisplayLabel();
  }

  @override
  void dispose() {
    assert(_platformSupportsAccelerators || _shortcutRegistryEntry == null);
    _displayLabel = '';
    if (_platformSupportsAccelerators) {
      _shortcutRegistryEntry?.dispose();
      _shortcutRegistryEntry = null;
      _shortcutRegistry = null;
      _anchor = null;
      HardwareKeyboard.instance.removeHandler(_listenToKeyEvent);
    }
    super.dispose();
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    if (!_platformSupportsAccelerators) {
      return;
    }
    _binding = MenuAcceleratorCallbackBinding.maybeOf(context);
    _anchor = _MenuAnchorState._maybeOf(context);
    _shortcutRegistry = ShortcutRegistry.maybeOf(context);
    _updateAcceleratorShortcut();
  }

  @override
  void didUpdateWidget(MenuAcceleratorLabel oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.label != oldWidget.label) {
      _updateDisplayLabel();
    }
  }

  static bool _altIsPressed() {
    return HardwareKeyboard.instance.logicalKeysPressed.intersection(<LogicalKeyboardKey>{
      LogicalKeyboardKey.altLeft,
      LogicalKeyboardKey.altRight,
      LogicalKeyboardKey.alt,
    }).isNotEmpty;
  }

  bool _listenToKeyEvent(KeyEvent event) {
    assert(_platformSupportsAccelerators);
    setState(() {
      _showAccelerators = _altIsPressed();
      _updateAcceleratorShortcut();
    });
    // Just listening, so it doesn't ever handle a key.
    return false;
  }

  void _updateAcceleratorShortcut() {
    assert(_platformSupportsAccelerators);
    _shortcutRegistryEntry?.dispose();
    _shortcutRegistryEntry = null;
    // Before registering an accelerator as a shortcut it should meet these
    // conditions:
    //
    // 1) Is showing accelerators (i.e. Alt key is down).
    // 2) Has an accelerator marker in the label.
    // 3) Has an associated action callback for the label (from the
    //    MenuAcceleratorCallbackBinding).
    // 4) Is part of an anchor that either doesn't have a submenu, or doesn't
    //    have any submenus currently open (only the "deepest" open menu should
    //    have accelerator shortcuts registered).
    if (_showAccelerators &&
        _acceleratorIndex != -1 &&
        _binding?.onInvoke != null &&
        (!_binding!.hasSubmenu || !(_anchor?._isOpen ?? false))) {
      final String acceleratorCharacter = _displayLabel[_acceleratorIndex].toLowerCase();
      _shortcutRegistryEntry = _shortcutRegistry?.addAll(<ShortcutActivator, Intent>{
        CharacterActivator(acceleratorCharacter, alt: true): VoidCallbackIntent(
          _binding!.onInvoke!,
        ),
      });
    }
  }

  void _updateDisplayLabel() {
    _displayLabel = MenuAcceleratorLabel.stripAcceleratorMarkers(
      widget.label,
      setIndex: (int index) {
        _acceleratorIndex = index;
      },
    );
  }

  @override
  Widget build(BuildContext context) {
    final int index = _showAccelerators ? _acceleratorIndex : -1;
    return widget.builder(context, _displayLabel, index);
  }
}

/// A label widget that is used as the label for a [MenuItemButton] or
/// [SubmenuButton].
///
/// It not only shows the [SubmenuButton.child] or [MenuItemButton.child], but if
/// there is a shortcut associated with the [MenuItemButton], it will display a
/// mnemonic for the shortcut. For [SubmenuButton]s, it will display a visual
/// indicator that there is a submenu.
class _MenuItemLabel extends StatelessWidget {
  /// Creates a const [_MenuItemLabel].
  ///
  /// The [child] and [hasSubmenu] arguments are required.
  const _MenuItemLabel({
    required this.hasSubmenu,
    this.showDecoration = true,
    this.leadingIcon,
    this.trailingIcon,
    this.shortcut,
    this.semanticsLabel,
    this.overflowAxis = Axis.vertical,
    this.submenuIcon,
    this.child,
  });

  /// Whether or not this menu has a submenu.
  ///
  /// Determines whether the submenu arrow is shown or not.
  final bool hasSubmenu;

  /// Whether or not this item should show decorations like shortcut labels or
  /// submenu arrows. Items in a [MenuBar] don't show these decorations when
  /// they are laid out horizontally.
  final bool showDecoration;

  /// The optional icon that comes before the [child].
  final Widget? leadingIcon;

  /// The optional icon that comes after the [child].
  final Widget? trailingIcon;

  /// The shortcut for this label, so that it can generate a string describing
  /// the shortcut.
  final MenuSerializableShortcut? shortcut;

  /// An optional Semantics label, which replaces the generated string when
  /// read by a screen reader.
  final String? semanticsLabel;

  /// The direction in which the menu item expands.
  final Axis overflowAxis;

  /// The submenu icon that is displayed when [showDecoration] and [hasSubmenu] are true.
  final Widget? submenuIcon;

  /// An optional child widget that is displayed in the label.
  final Widget? child;

  @override
  Widget build(BuildContext context) {
    final VisualDensity density = Theme.of(context).visualDensity;
    final double horizontalPadding = math.max(
      _kLabelItemMinSpacing,
      _kLabelItemDefaultSpacing + density.horizontal * 2,
    );
    Widget leadings;
    if (overflowAxis == Axis.vertical) {
      leadings = Expanded(
        child: ClipRect(
          child: Row(
            mainAxisSize: MainAxisSize.min,
            children: <Widget>[
              if (leadingIcon != null) leadingIcon!,
              if (child != null)
                Expanded(
                  child: ClipRect(
                    child: Padding(
                      padding:
                          leadingIcon != null
                              ? EdgeInsetsDirectional.only(start: horizontalPadding)
                              : EdgeInsets.zero,
                      child: child,
                    ),
                  ),
                ),
            ],
          ),
        ),
      );
    } else {
      leadings = Row(
        mainAxisSize: MainAxisSize.min,
        children: <Widget>[
          if (leadingIcon != null) leadingIcon!,
          if (child != null)
            Padding(
              padding:
                  leadingIcon != null
                      ? EdgeInsetsDirectional.only(start: horizontalPadding)
                      : EdgeInsets.zero,
              child: child,
            ),
        ],
      );
    }

    Widget menuItemLabel = Row(
      mainAxisAlignment: MainAxisAlignment.spaceBetween,
      children: <Widget>[
        leadings,
        if (trailingIcon != null)
          Padding(
            padding: EdgeInsetsDirectional.only(start: horizontalPadding),
            child: trailingIcon,
          ),
        if (showDecoration && shortcut != null)
          Padding(
            padding: EdgeInsetsDirectional.only(start: horizontalPadding),
            child: Text(
              _LocalizedShortcutLabeler.instance.getShortcutLabel(
                shortcut!,
                MaterialLocalizations.of(context),
              ),
            ),
          ),
        if (showDecoration && hasSubmenu)
          Padding(
            padding: EdgeInsetsDirectional.only(start: horizontalPadding),
            child: submenuIcon,
          ),
      ],
    );
    if (semanticsLabel != null) {
      menuItemLabel = Semantics(
        label: semanticsLabel,
        excludeSemantics: true,
        child: menuItemLabel,
      );
    }
    return menuItemLabel;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      DiagnosticsProperty<MenuSerializableShortcut>('shortcut', shortcut, defaultValue: null),
    );
    properties.add(DiagnosticsProperty<bool>('hasSubmenu', hasSubmenu));
    properties.add(DiagnosticsProperty<bool>('showDecoration', showDecoration));
  }
}

// Positions the menu in the view while trying to keep as much as possible
// visible in the view.
class _MenuLayout extends SingleChildLayoutDelegate {
  const _MenuLayout({
    required this.anchorRect,
    required this.textDirection,
    required this.alignment,
    required this.alignmentOffset,
    required this.menuPosition,
    required this.menuPadding,
    required this.avoidBounds,
    required this.orientation,
    required this.parentOrientation,
  });

  // Rectangle of underlying button, relative to the overlay's dimensions.
  final Rect anchorRect;

  // Whether to prefer going to the left or to the right.
  final TextDirection textDirection;

  // The alignment to use when finding the ideal location for the menu.
  final AlignmentGeometry alignment;

  // The offset from the alignment position to find the ideal location for the
  // menu.
  final Offset alignmentOffset;

  // The position passed to the open method, if any.
  final Offset? menuPosition;

  // The padding on the inside of the menu, so it can be accounted for when
  // positioning.
  final EdgeInsetsGeometry menuPadding;

  // List of rectangles that we should avoid overlapping. Unusable screen area.
  final Set<Rect> avoidBounds;

  // The orientation of this menu.
  final Axis orientation;

  // The orientation of this menu's parent.
  final Axis parentOrientation;

  @override
  BoxConstraints getConstraintsForChild(BoxConstraints constraints) {
    // The menu can be at most the size of the overlay minus _kMenuViewPadding
    // pixels in each direction.
    return BoxConstraints.loose(
      constraints.biggest,
    ).deflate(const EdgeInsets.all(_kMenuViewPadding));
  }

  @override
  Offset getPositionForChild(Size size, Size childSize) {
    // size: The size of the overlay.
    // childSize: The size of the menu, when fully open, as determined by
    // getConstraintsForChild.
    final Rect overlayRect = Offset.zero & size;
    double x;
    double y;
    if (menuPosition == null) {
      Offset desiredPosition = alignment.resolve(textDirection).withinRect(anchorRect);
      final Offset directionalOffset;
      if (alignment is AlignmentDirectional) {
        directionalOffset = switch (textDirection) {
          TextDirection.rtl => Offset(-alignmentOffset.dx, alignmentOffset.dy),
          TextDirection.ltr => alignmentOffset,
        };
      } else {
        directionalOffset = alignmentOffset;
      }
      desiredPosition += directionalOffset;
      x = desiredPosition.dx;
      y = desiredPosition.dy;
      switch (textDirection) {
        case TextDirection.rtl:
          x -= childSize.width;
        case TextDirection.ltr:
          break;
      }
    } else {
      final Offset adjustedPosition = menuPosition! + anchorRect.topLeft;
      x = adjustedPosition.dx;
      y = adjustedPosition.dy;
    }

    final Iterable<Rect> subScreens = DisplayFeatureSubScreen.subScreensInBounds(
      overlayRect,
      avoidBounds,
    );
    final Rect allowedRect = _closestScreen(subScreens, anchorRect.center);
    bool offLeftSide(double x) => x < allowedRect.left;
    bool offRightSide(double x) => x + childSize.width > allowedRect.right;
    bool offTop(double y) => y < allowedRect.top;
    bool offBottom(double y) => y + childSize.height > allowedRect.bottom;
    // Avoid going outside an area defined as the rectangle offset from the
    // edge of the screen by the button padding. If the menu is off of the screen,
    // move the menu to the other side of the button first, and then if it
    // doesn't fit there, then just move it over as much as needed to make it
    // fit.
    if (childSize.width >= allowedRect.width) {
      // It just doesn't fit, so put as much on the screen as possible.
      x = allowedRect.left;
    } else {
      if (offLeftSide(x)) {
        // If the parent is a different orientation than the current one, then
        // just push it over instead of trying the other side.
        if (parentOrientation != orientation) {
          x = allowedRect.left;
        } else {
          final double newX = anchorRect.right + alignmentOffset.dx;
          if (!offRightSide(newX)) {
            x = newX;
          } else {
            x = allowedRect.left;
          }
        }
      } else if (offRightSide(x)) {
        if (parentOrientation != orientation) {
          x = allowedRect.right - childSize.width;
        } else {
          final double newX = anchorRect.left - childSize.width - alignmentOffset.dx;
          if (!offLeftSide(newX)) {
            x = newX;
          } else {
            x = allowedRect.right - childSize.width;
          }
        }
      }
    }
    if (childSize.height >= allowedRect.height) {
      // Too tall to fit, fit as much on as possible.
      y = allowedRect.top;
    } else {
      if (offTop(y)) {
        final double newY = anchorRect.bottom;
        if (!offBottom(newY)) {
          y = newY;
        } else {
          y = allowedRect.top;
        }
      } else if (offBottom(y)) {
        final double newY = anchorRect.top - childSize.height;
        if (!offTop(newY)) {
          // Only move the menu up if its parent is horizontal (MenuAnchor/MenuBar).
          if (parentOrientation == Axis.horizontal) {
            y = newY - alignmentOffset.dy;
          } else {
            y = newY;
          }
        } else {
          y = allowedRect.bottom - childSize.height;
        }
      }
    }
    return Offset(x, y);
  }

  @override
  bool shouldRelayout(_MenuLayout oldDelegate) {
    return anchorRect != oldDelegate.anchorRect ||
        textDirection != oldDelegate.textDirection ||
        alignment != oldDelegate.alignment ||
        alignmentOffset != oldDelegate.alignmentOffset ||
        menuPosition != oldDelegate.menuPosition ||
        menuPadding != oldDelegate.menuPadding ||
        orientation != oldDelegate.orientation ||
        parentOrientation != oldDelegate.parentOrientation ||
        !setEquals(avoidBounds, oldDelegate.avoidBounds);
  }

  Rect _closestScreen(Iterable<Rect> screens, Offset point) {
    Rect closest = screens.first;
    for (final Rect screen in screens) {
      if ((screen.center - point).distance < (closest.center - point).distance) {
        closest = screen;
      }
    }
    return closest;
  }
}

/// A widget that manages a list of menu buttons in a menu.
///
/// It sizes itself to the widest/tallest item it contains, and then sizes all
/// the other entries to match.
class _MenuPanel extends StatefulWidget {
  const _MenuPanel({
    required this.menuStyle,
    this.clipBehavior = Clip.none,
    required this.orientation,
    this.crossAxisUnconstrained = true,
    required this.children,
  });

  /// The menu style that has all the attributes for this menu panel.
  final MenuStyle? menuStyle;

  /// {@macro flutter.material.Material.clipBehavior}
  ///
  /// Defaults to [Clip.none].
  final Clip clipBehavior;

  /// Determine if a [UnconstrainedBox] can be applied to the menu panel to allow it to render
  /// at its "natural" size.
  ///
  /// Defaults to true. When it is set to false, it can be useful when the menu should
  /// be constrained in both main-axis and cross-axis, such as a [DropdownMenu].
  final bool crossAxisUnconstrained;

  /// The layout orientation of this panel.
  final Axis orientation;

  /// The list of widgets to use as children of this menu panel.
  ///
  /// These are the top level [SubmenuButton]s.
  final List<Widget> children;

  @override
  State<_MenuPanel> createState() => _MenuPanelState();
}

class _MenuPanelState extends State<_MenuPanel> {
  ScrollController scrollController = ScrollController();

  @override
  void dispose() {
    scrollController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final (MenuStyle? themeStyle, MenuStyle defaultStyle) = switch (widget.orientation) {
      Axis.horizontal => (MenuBarTheme.of(context).style, _MenuBarDefaultsM3(context)),
      Axis.vertical => (MenuTheme.of(context).style, _MenuDefaultsM3(context)),
    };
    final MenuStyle? widgetStyle = widget.menuStyle;

    T? effectiveValue<T>(T? Function(MenuStyle? style) getProperty) {
      return getProperty(widgetStyle) ?? getProperty(themeStyle) ?? getProperty(defaultStyle);
    }

    T? resolve<T>(MaterialStateProperty<T>? Function(MenuStyle? style) getProperty) {
      return effectiveValue((MenuStyle? style) {
        return getProperty(style)?.resolve(<MaterialState>{});
      });
    }

    final Color? backgroundColor = resolve<Color?>((MenuStyle? style) => style?.backgroundColor);
    final Color? shadowColor = resolve<Color?>((MenuStyle? style) => style?.shadowColor);
    final Color? surfaceTintColor = resolve<Color?>((MenuStyle? style) => style?.surfaceTintColor);
    final double elevation = resolve<double?>((MenuStyle? style) => style?.elevation) ?? 0;
    final Size? minimumSize = resolve<Size?>((MenuStyle? style) => style?.minimumSize);
    final Size? fixedSize = resolve<Size?>((MenuStyle? style) => style?.fixedSize);
    final Size? maximumSize = resolve<Size?>((MenuStyle? style) => style?.maximumSize);
    final BorderSide? side = resolve<BorderSide?>((MenuStyle? style) => style?.side);
    final OutlinedBorder shape = resolve<OutlinedBorder?>(
      (MenuStyle? style) => style?.shape,
    )!.copyWith(side: side);
    final VisualDensity visualDensity =
        effectiveValue((MenuStyle? style) => style?.visualDensity) ?? VisualDensity.standard;
    final EdgeInsetsGeometry padding =
        resolve<EdgeInsetsGeometry?>((MenuStyle? style) => style?.padding) ?? EdgeInsets.zero;
    final Offset densityAdjustment = visualDensity.baseSizeAdjustment;
    // Per the Material Design team: don't allow the VisualDensity
    // adjustment to reduce the width of the left/right padding. If we
    // did, VisualDensity.compact, the default for desktop/web, would
    // reduce the horizontal padding to zero.
    final double dy = densityAdjustment.dy;
    final double dx = math.max(0, densityAdjustment.dx);
    final EdgeInsetsGeometry resolvedPadding = padding
        .add(EdgeInsets.symmetric(horizontal: dx, vertical: dy))
        .clamp(EdgeInsets.zero, EdgeInsetsGeometry.infinity);

    BoxConstraints effectiveConstraints = visualDensity.effectiveConstraints(
      BoxConstraints(
        minWidth: minimumSize?.width ?? 0,
        minHeight: minimumSize?.height ?? 0,
        maxWidth: maximumSize?.width ?? double.infinity,
        maxHeight: maximumSize?.height ?? double.infinity,
      ),
    );
    if (fixedSize != null) {
      final Size size = effectiveConstraints.constrain(fixedSize);
      if (size.width.isFinite) {
        effectiveConstraints = effectiveConstraints.copyWith(
          minWidth: size.width,
          maxWidth: size.width,
        );
      }
      if (size.height.isFinite) {
        effectiveConstraints = effectiveConstraints.copyWith(
          minHeight: size.height,
          maxHeight: size.height,
        );
      }
    }

    // If the menu panel is horizontal, then the children should be wrapped in
    // an IntrinsicWidth widget to ensure that the children are as wide as the
    // widest child.
    List<Widget> children = widget.children;
    if (widget.orientation == Axis.horizontal) {
      children =
          children.map<Widget>((Widget child) {
            return IntrinsicWidth(child: child);
          }).toList();
    }

    Widget menuPanel = _intrinsicCrossSize(
      child: Material(
        elevation: elevation,
        shape: shape,
        color: backgroundColor,
        shadowColor: shadowColor,
        surfaceTintColor: surfaceTintColor,
        type: backgroundColor == null ? MaterialType.transparency : MaterialType.canvas,
        clipBehavior: widget.clipBehavior,
        child: Padding(
          padding: resolvedPadding,
          child: ScrollConfiguration(
            behavior: ScrollConfiguration.of(context).copyWith(
              scrollbars: false,
              overscroll: false,
              physics: const ClampingScrollPhysics(),
            ),
            child: PrimaryScrollController(
              controller: scrollController,
              child: Scrollbar(
                thumbVisibility: true,
                child: SingleChildScrollView(
                  controller: scrollController,
                  scrollDirection: widget.orientation,
                  child: Flex(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    textDirection: Directionality.of(context),
                    direction: widget.orientation,
                    mainAxisSize: MainAxisSize.min,
                    children: children,
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );

    if (widget.crossAxisUnconstrained) {
      menuPanel = UnconstrainedBox(
        constrainedAxis: widget.orientation,
        clipBehavior: Clip.hardEdge,
        alignment: AlignmentDirectional.centerStart,
        child: menuPanel,
      );
    }

    return ConstrainedBox(constraints: effectiveConstraints, child: menuPanel);
  }

  Widget _intrinsicCrossSize({required Widget child}) {
    return switch (widget.orientation) {
      Axis.horizontal => IntrinsicHeight(child: child),
      Axis.vertical => IntrinsicWidth(child: child),
    };
  }
}

// A widget that defines the menu drawn in the overlay.
class _Submenu extends StatelessWidget {
  const _Submenu({
    required this.anchor,
    required this.layerLink,
    required this.menuStyle,
    required this.menuPosition,
    required this.alignmentOffset,
    required this.clipBehavior,
    this.crossAxisUnconstrained = true,
    required this.menuChildren,
  });

  final _MenuAnchorState anchor;
  final LayerLink? layerLink;
  final MenuStyle? menuStyle;
  final Offset? menuPosition;
  final Offset alignmentOffset;
  final Clip clipBehavior;
  final bool crossAxisUnconstrained;
  final List<Widget> menuChildren;

  @override
  Widget build(BuildContext context) {
    // Use the text direction of the context where the button is.
    final TextDirection textDirection = Directionality.of(context);
    final (MenuStyle? themeStyle, MenuStyle defaultStyle) = switch (anchor._parent?._orientation) {
      Axis.horizontal || null => (MenuBarTheme.of(context).style, _MenuBarDefaultsM3(context)),
      Axis.vertical => (MenuTheme.of(context).style, _MenuDefaultsM3(context)),
    };
    T? effectiveValue<T>(T? Function(MenuStyle? style) getProperty) {
      return getProperty(menuStyle) ?? getProperty(themeStyle) ?? getProperty(defaultStyle);
    }

    T? resolve<T>(MaterialStateProperty<T>? Function(MenuStyle? style) getProperty) {
      return effectiveValue((MenuStyle? style) {
        return getProperty(style)?.resolve(<MaterialState>{});
      });
    }

    final MaterialStateMouseCursor mouseCursor = _MouseCursor(
      (Set<MaterialState> states) =>
          effectiveValue((MenuStyle? style) => style?.mouseCursor?.resolve(states)),
    );

    final VisualDensity visualDensity =
        effectiveValue((MenuStyle? style) => style?.visualDensity) ??
        Theme.of(context).visualDensity;
    final AlignmentGeometry alignment = effectiveValue((MenuStyle? style) => style?.alignment)!;
    final EdgeInsetsGeometry padding =
        resolve<EdgeInsetsGeometry?>((MenuStyle? style) => style?.padding) ?? EdgeInsets.zero;
    final Offset densityAdjustment = visualDensity.baseSizeAdjustment;
    // Per the Material Design team: don't allow the VisualDensity
    // adjustment to reduce the width of the left/right padding. If we
    // did, VisualDensity.compact, the default for desktop/web, would
    // reduce the horizontal padding to zero.
    final double dy = densityAdjustment.dy;
    final double dx = math.max(0, densityAdjustment.dx);
    final EdgeInsetsGeometry resolvedPadding = padding
        .add(EdgeInsets.fromLTRB(dx, dy, dx, dy))
        .clamp(EdgeInsets.zero, EdgeInsetsGeometry.infinity);
    final BuildContext anchorContext = anchor._anchorKey.currentContext!;
    final RenderBox overlay = Overlay.of(anchorContext).context.findRenderObject()! as RenderBox;

    Offset upperLeft = Offset.zero;
    Offset bottomRight = Offset.zero;
    if (layerLink == null) {
      final RenderBox anchorBox = anchorContext.findRenderObject()! as RenderBox;
      upperLeft = anchorBox.localToGlobal(Offset(dx, -dy), ancestor: overlay);
      bottomRight = anchorBox.localToGlobal(anchorBox.paintBounds.bottomRight, ancestor: overlay);
    }
    final Rect anchorRect = Rect.fromPoints(upperLeft, bottomRight);

    Widget child = Theme(
      data: Theme.of(context).copyWith(visualDensity: visualDensity),
      child: ConstrainedBox(
        constraints: BoxConstraints.loose(overlay.paintBounds.size),
        child: CustomSingleChildLayout(
          delegate: _MenuLayout(
            anchorRect: anchorRect,
            textDirection: textDirection,
            avoidBounds: DisplayFeatureSubScreen.avoidBounds(MediaQuery.of(context)).toSet(),
            menuPadding: resolvedPadding,
            alignment: alignment,
            alignmentOffset: alignmentOffset,
            menuPosition: menuPosition,
            orientation: anchor._orientation,
            parentOrientation: anchor._parent?._orientation ?? Axis.horizontal,
          ),
          child: TapRegion(
            groupId: anchor._root,
            consumeOutsideTaps: anchor._root._isOpen && anchor.widget.consumeOutsideTap,
            onTapOutside: (PointerDownEvent event) {
              anchor._close();
            },
            child: MouseRegion(
              cursor: mouseCursor,
              hitTestBehavior: HitTestBehavior.deferToChild,
              child: FocusScope(
                node: anchor._menuScopeNode,
                skipTraversal: true,
                child: Actions(
                  actions: <Type, Action<Intent>>{
                    DismissIntent: DismissMenuAction(controller: anchor._menuController),
                  },
                  child: Shortcuts(
                    shortcuts: _kMenuTraversalShortcuts,
                    child: _MenuPanel(
                      menuStyle: menuStyle,
                      clipBehavior: clipBehavior,
                      orientation: anchor._orientation,
                      crossAxisUnconstrained: crossAxisUnconstrained,
                      children: menuChildren,
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    );

    if (layerLink != null) {
      child = CompositedTransformFollower(
        link: layerLink!,
        targetAnchor: Alignment.bottomLeft,
        child: child,
      );
    }

    return child;
  }
}

/// Wraps the [WidgetStateMouseCursor] so that it can default to
/// [MouseCursor.uncontrolled] if none is set.
class _MouseCursor extends MaterialStateMouseCursor {
  const _MouseCursor(this.resolveCallback);

  final MaterialPropertyResolver<MouseCursor?> resolveCallback;

  @override
  MouseCursor resolve(Set<MaterialState> states) =>
      resolveCallback(states) ?? MouseCursor.uncontrolled;

  @override
  String get debugDescription => 'Menu_MouseCursor';
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

/// Whether [defaultTargetPlatform] is an Apple platform (Mac or iOS).
bool get _isCupertino {
  switch (defaultTargetPlatform) {
    case TargetPlatform.iOS:
    case TargetPlatform.macOS:
      return true;
    case TargetPlatform.android:
    case TargetPlatform.fuchsia:
    case TargetPlatform.linux:
    case TargetPlatform.windows:
      return false;
  }
}

/// Whether [defaultTargetPlatform] is one that uses symbolic shortcuts.
///
/// Mac and iOS use special symbols for modifier keys instead of their names,
/// render them in a particular order defined by Apple's human interface
/// guidelines, and format them so that the modifier keys always align.
bool get _usesSymbolicModifiers {
  return _isCupertino;
}

bool get _platformSupportsAccelerators {
  // On iOS and macOS, pressing the Option key (a.k.a. the Alt key) causes a
  // different set of characters to be generated, and the native menus don't
  // support accelerators anyhow, so we just disable accelerators on these
  // platforms.
  return !_isCupertino;
}

// BEGIN GENERATED TOKEN PROPERTIES - Menu

// Do not edit by hand. The code between the "BEGIN GENERATED" and
// "END GENERATED" comments are generated from data in the Material
// Design token database by the script:
//   dev/tools/gen_defaults/bin/gen_defaults.dart.

// dart format off
class _MenuBarDefaultsM3 extends MenuStyle {
  _MenuBarDefaultsM3(this.context)
    : super(
      elevation: const MaterialStatePropertyAll<double?>(3.0),
      shape: const MaterialStatePropertyAll<OutlinedBorder>(_defaultMenuBorder),
      alignment: AlignmentDirectional.bottomStart,
    );

  static const RoundedRectangleBorder _defaultMenuBorder =
    RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4.0)));

  final BuildContext context;

  late final ColorScheme _colors = Theme.of(context).colorScheme;

  @override
  MaterialStateProperty<Color?> get backgroundColor {
    return MaterialStatePropertyAll<Color?>(_colors.surfaceContainer);
  }

  @override
  MaterialStateProperty<Color?>? get shadowColor {
    return MaterialStatePropertyAll<Color?>(_colors.shadow);
  }

  @override
  MaterialStateProperty<Color?>? get surfaceTintColor {
    return const MaterialStatePropertyAll<Color?>(Colors.transparent);
  }

  @override
  MaterialStateProperty<EdgeInsetsGeometry?>? get padding {
    return const MaterialStatePropertyAll<EdgeInsetsGeometry>(
      EdgeInsetsDirectional.symmetric(
        horizontal: _kTopLevelMenuHorizontalMinPadding
      ),
    );
  }

  @override
  VisualDensity get visualDensity => Theme.of(context).visualDensity;
}

class _MenuButtonDefaultsM3 extends ButtonStyle {
  _MenuButtonDefaultsM3(this.context)
    : super(
      animationDuration: kThemeChangeDuration,
      enableFeedback: true,
      alignment: AlignmentDirectional.centerStart,
    );

  final BuildContext context;

  late final ColorScheme _colors = Theme.of(context).colorScheme;
  late final TextTheme _textTheme = Theme.of(context).textTheme;

  @override
  MaterialStateProperty<Color?>? get backgroundColor {
    return ButtonStyleButton.allOrNull<Color>(Colors.transparent);
  }

  // No default shadow color

  // No default surface tint color

  @override
  MaterialStateProperty<double>? get elevation {
    return ButtonStyleButton.allOrNull<double>(0.0);
  }

  @override
  MaterialStateProperty<Color?>? get foregroundColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return _colors.onSurface.withOpacity(0.38);
      }
      if (states.contains(MaterialState.pressed)) {
        return _colors.onSurface;
      }
      if (states.contains(MaterialState.hovered)) {
        return _colors.onSurface;
      }
      if (states.contains(MaterialState.focused)) {
        return _colors.onSurface;
      }
      return _colors.onSurface;
    });
  }

  @override
  MaterialStateProperty<Color?>? get iconColor {
    return MaterialStateProperty.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return _colors.onSurface.withOpacity(0.38);
      }
      if (states.contains(MaterialState.pressed)) {
        return _colors.onSurfaceVariant;
      }
      if (states.contains(MaterialState.hovered)) {
        return _colors.onSurfaceVariant;
      }
      if (states.contains(MaterialState.focused)) {
        return _colors.onSurfaceVariant;
      }
      return _colors.onSurfaceVariant;
    });
  }

  // No default fixedSize

  @override
  MaterialStateProperty<double>? get iconSize {
    return const MaterialStatePropertyAll<double>(24.0);
  }

  @override
  MaterialStateProperty<Size>? get maximumSize {
    return ButtonStyleButton.allOrNull<Size>(Size.infinite);
  }

  @override
  MaterialStateProperty<Size>? get minimumSize {
    return ButtonStyleButton.allOrNull<Size>(const Size(64.0, 48.0));
  }

  @override
  MaterialStateProperty<MouseCursor?>? get mouseCursor {
    return MaterialStateProperty.resolveWith(
      (Set<MaterialState> states) {
        if (states.contains(MaterialState.disabled)) {
          return SystemMouseCursors.basic;
        }
        return SystemMouseCursors.click;
      },
    );
  }

  @override
  MaterialStateProperty<Color?>? get overlayColor {
    return MaterialStateProperty.resolveWith(
      (Set<MaterialState> states) {
        if (states.contains(MaterialState.pressed)) {
          return _colors.onSurface.withOpacity(0.1);
        }
        if (states.contains(MaterialState.hovered)) {
          return _colors.onSurface.withOpacity(0.08);
        }
        if (states.contains(MaterialState.focused)) {
          return _colors.onSurface.withOpacity(0.1);
        }
        return Colors.transparent;
      },
    );
  }

  @override
  MaterialStateProperty<EdgeInsetsGeometry>? get padding {
    return ButtonStyleButton.allOrNull<EdgeInsetsGeometry>(_scaledPadding(context));
  }

  // No default side

  @override
  MaterialStateProperty<OutlinedBorder>? get shape {
    return ButtonStyleButton.allOrNull<OutlinedBorder>(const RoundedRectangleBorder());
  }

  @override
  InteractiveInkFeatureFactory? get splashFactory => Theme.of(context).splashFactory;

  @override
  MaterialTapTargetSize? get tapTargetSize => Theme.of(context).materialTapTargetSize;

  @override
  MaterialStateProperty<TextStyle?> get textStyle {
    // TODO(tahatesser): This is taken from https://m3.material.io/components/menus/specs
    // Update this when the token is available.
    return MaterialStatePropertyAll<TextStyle?>(_textTheme.labelLarge);
  }

  @override
  VisualDensity? get visualDensity => Theme.of(context).visualDensity;

  // The horizontal padding number comes from the spec.
  EdgeInsetsGeometry _scaledPadding(BuildContext context) {
    VisualDensity visualDensity = Theme.of(context).visualDensity;
    // When horizontal VisualDensity is greater than zero, set it to zero
    // because the [ButtonStyleButton] has already handle the padding based on the density.
    // However, the [ButtonStyleButton] doesn't allow the [VisualDensity] adjustment
    // to reduce the width of the left/right padding, so we need to handle it here if
    // the density is less than zero, such as on desktop platforms.
    if (visualDensity.horizontal > 0) {
      visualDensity = VisualDensity(vertical: visualDensity.vertical);
    }
    // Since the threshold paddings used below are empirical values determined
    // at a font size of 14.0, 14.0 is used as the base value for scaling the
    // padding.
    final double fontSize = Theme.of(context).textTheme.labelLarge?.fontSize ?? 14.0;
    final double fontSizeRatio = MediaQuery.textScalerOf(context).scale(fontSize) / 14.0;
    return ButtonStyleButton.scaledPadding(
      EdgeInsets.symmetric(horizontal: math.max(
        _kMenuViewPadding,
        _kLabelItemDefaultSpacing + visualDensity.baseSizeAdjustment.dx,
      )),
      EdgeInsets.symmetric(horizontal: math.max(
        _kMenuViewPadding,
        8 + visualDensity.baseSizeAdjustment.dx,
      )),
      const EdgeInsets.symmetric(horizontal: _kMenuViewPadding),
      fontSizeRatio,
    );
  }
}

class _MenuDefaultsM3 extends MenuStyle {
  _MenuDefaultsM3(this.context)
    : super(
      elevation: const MaterialStatePropertyAll<double?>(3.0),
      shape: const MaterialStatePropertyAll<OutlinedBorder>(_defaultMenuBorder),
      alignment: AlignmentDirectional.topEnd,
    );

  static const RoundedRectangleBorder _defaultMenuBorder =
    RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4.0)));

  final BuildContext context;

  late final ColorScheme _colors = Theme.of(context).colorScheme;

  @override
  MaterialStateProperty<Color?> get backgroundColor {
    return MaterialStatePropertyAll<Color?>(_colors.surfaceContainer);
  }

  @override
  MaterialStateProperty<Color?>? get surfaceTintColor {
    return const MaterialStatePropertyAll<Color?>(Colors.transparent);
  }

  @override
  MaterialStateProperty<Color?>? get shadowColor {
    return MaterialStatePropertyAll<Color?>(_colors.shadow);
  }

  @override
  MaterialStateProperty<EdgeInsetsGeometry?>? get padding {
    return const MaterialStatePropertyAll<EdgeInsetsGeometry>(
      EdgeInsetsDirectional.symmetric(vertical: _kMenuVerticalMinPadding),
    );
  }

  @override
  VisualDensity get visualDensity => Theme.of(context).visualDensity;
}
// dart format on

// END GENERATED TOKEN PROPERTIES - Menu
