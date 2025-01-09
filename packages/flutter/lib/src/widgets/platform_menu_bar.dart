// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';

import 'actions.dart';
import 'basic.dart';
import 'binding.dart';
import 'focus_manager.dart';
import 'framework.dart';
import 'shortcuts.dart';

// "flutter/menu" Method channel methods.
const String _kMenuSetMethod = 'Menu.setMenus';
const String _kMenuSelectedCallbackMethod = 'Menu.selectedCallback';
const String _kMenuItemOpenedMethod = 'Menu.opened';
const String _kMenuItemClosedMethod = 'Menu.closed';

// Keys for channel communication map.
const String _kIdKey = 'id';
const String _kLabelKey = 'label';
const String _kEnabledKey = 'enabled';
const String _kChildrenKey = 'children';
const String _kIsDividerKey = 'isDivider';
const String _kPlatformDefaultMenuKey = 'platformProvidedMenu';
const String _kShortcutCharacter = 'shortcutCharacter';
const String _kShortcutTrigger = 'shortcutTrigger';
const String _kShortcutModifiers = 'shortcutModifiers';

/// A class used by [MenuSerializableShortcut] to describe the shortcut for
/// serialization to send to the platform for rendering a [PlatformMenuBar].
///
/// See also:
///
///  * [PlatformMenuBar], a widget that defines a menu bar for the platform to
///    render natively.
///  * [MenuSerializableShortcut], a mixin allowing a [ShortcutActivator] to
///    provide data for serialization of the shortcut for sending to the
///    platform.
class ShortcutSerialization {
  /// Creates a [ShortcutSerialization] representing a single character.
  ///
  /// This is used by a [CharacterActivator] to serialize itself.
  ShortcutSerialization.character(
    String character, {
    bool alt = false,
    bool control = false,
    bool meta = false,
  }) : assert(character.length == 1),
       _character = character,
       _trigger = null,
       _alt = alt,
       _control = control,
       _meta = meta,
       _shift = null,
       _internal = <String, Object?>{
         _kShortcutCharacter: character,
         _kShortcutModifiers:
             (control ? _shortcutModifierControl : 0) |
             (alt ? _shortcutModifierAlt : 0) |
             (meta ? _shortcutModifierMeta : 0),
       };

  /// Creates a [ShortcutSerialization] representing a specific
  /// [LogicalKeyboardKey] and modifiers.
  ///
  /// This is used by a [SingleActivator] to serialize itself.
  ShortcutSerialization.modifier(
    LogicalKeyboardKey trigger, {
    bool alt = false,
    bool control = false,
    bool meta = false,
    bool shift = false,
  }) : assert(
         trigger != LogicalKeyboardKey.alt &&
             trigger != LogicalKeyboardKey.altLeft &&
             trigger != LogicalKeyboardKey.altRight &&
             trigger != LogicalKeyboardKey.control &&
             trigger != LogicalKeyboardKey.controlLeft &&
             trigger != LogicalKeyboardKey.controlRight &&
             trigger != LogicalKeyboardKey.meta &&
             trigger != LogicalKeyboardKey.metaLeft &&
             trigger != LogicalKeyboardKey.metaRight &&
             trigger != LogicalKeyboardKey.shift &&
             trigger != LogicalKeyboardKey.shiftLeft &&
             trigger != LogicalKeyboardKey.shiftRight,
         'Specifying a modifier key as a trigger is not allowed. '
         'Use provided boolean parameters instead.',
       ),
       _trigger = trigger,
       _character = null,
       _alt = alt,
       _control = control,
       _meta = meta,
       _shift = shift,
       _internal = <String, Object?>{
         _kShortcutTrigger: trigger.keyId,
         _kShortcutModifiers:
             (alt ? _shortcutModifierAlt : 0) |
             (control ? _shortcutModifierControl : 0) |
             (meta ? _shortcutModifierMeta : 0) |
             (shift ? _shortcutModifierShift : 0),
       };

  final Map<String, Object?> _internal;

  /// The keyboard key that triggers this shortcut, if any.
  LogicalKeyboardKey? get trigger => _trigger;
  final LogicalKeyboardKey? _trigger;

  /// The character that triggers this shortcut, if any.
  String? get character => _character;
  final String? _character;

  /// If this shortcut has a [trigger], this indicates whether or not the
  /// alt modifier needs to be down or not.
  bool? get alt => _alt;
  final bool? _alt;

  /// If this shortcut has a [trigger], this indicates whether or not the
  /// control modifier needs to be down or not.
  bool? get control => _control;
  final bool? _control;

  /// If this shortcut has a [trigger], this indicates whether or not the meta
  /// (also known as the Windows or Command key) modifier needs to be down or
  /// not.
  bool? get meta => _meta;
  final bool? _meta;

  /// If this shortcut has a [trigger], this indicates whether or not the
  /// shift modifier needs to be down or not.
  bool? get shift => _shift;
  final bool? _shift;

  /// The bit mask for the [LogicalKeyboardKey.alt] key (or it's left/right
  /// equivalents) being down.
  static const int _shortcutModifierAlt = 1 << 2;

  /// The bit mask for the [LogicalKeyboardKey.control] key (or it's left/right
  /// equivalents) being down.
  static const int _shortcutModifierControl = 1 << 3;

  /// The bit mask for the [LogicalKeyboardKey.meta] key (or it's left/right
  /// equivalents) being down.
  static const int _shortcutModifierMeta = 1 << 0;

  /// The bit mask for the [LogicalKeyboardKey.shift] key (or it's left/right
  /// equivalents) being down.
  static const int _shortcutModifierShift = 1 << 1;

  /// Converts the internal representation to the format needed for a
  /// [PlatformMenuItem] to include it in its serialized form for sending to the
  /// platform.
  Map<String, Object?> toChannelRepresentation() => _internal;
}

/// A mixin allowing a [ShortcutActivator] to provide data for serialization of
/// the shortcut when sending to the platform.
///
/// This is meant for those who have written their own [ShortcutActivator]
/// subclass, and would like to have it work for menus in a [PlatformMenuBar] as
/// well.
///
/// Keep in mind that there are limits to the capabilities of the platform APIs,
/// and not all kinds of [ShortcutActivator]s will work with them.
///
/// See also:
///
///  * [SingleActivator], a [ShortcutActivator] which implements this mixin.
///  * [CharacterActivator], another [ShortcutActivator] which implements this mixin.
mixin MenuSerializableShortcut implements ShortcutActivator {
  /// Implement this in a [ShortcutActivator] subclass to allow it to be
  /// serialized for use in a [PlatformMenuBar].
  ShortcutSerialization serializeForMenu();
}

/// An abstract delegate class that can be used to set
/// [WidgetsBinding.platformMenuDelegate] to provide for managing platform
/// menus.
///
/// This can be subclassed to provide a different menu plugin than the default
/// system-provided plugin for managing [PlatformMenuBar] menus.
///
/// The [setMenus] method allows for setting of the menu hierarchy when the
/// [PlatformMenuBar] menu hierarchy changes.
///
/// This delegate doesn't handle the results of clicking on a menu item, which
/// is left to the implementor of subclasses of [PlatformMenuDelegate] to
/// handle for their implementation.
///
/// This delegate typically knows how to serialize a [PlatformMenu]
/// hierarchy, send it over a channel, and register for calls from the channel
/// when a menu is invoked or a submenu is opened or closed.
///
/// See [DefaultPlatformMenuDelegate] for an example of implementing one of
/// these.
///
/// See also:
///
///  * [PlatformMenuBar], the widget that adds a platform menu bar to an
///    application, and uses [setMenus] to send the menus to the platform.
///  * [PlatformMenu], the class that describes a menu item with children
///    that appear in a cascading menu.
///  * [PlatformMenuItem], the class that describes the leaves of a menu
///    hierarchy.
abstract class PlatformMenuDelegate {
  /// A const constructor so that subclasses can have const constructors.
  const PlatformMenuDelegate();

  /// Sets the entire menu hierarchy for a platform-rendered menu bar.
  ///
  /// The `topLevelMenus` argument is the list of menus that appear in the menu
  /// bar, which themselves can have children.
  ///
  /// To update the menu hierarchy or menu item state, call [setMenus] with the
  /// modified hierarchy, and it will overwrite the previous menu state.
  ///
  /// See also:
  ///
  ///  * [PlatformMenuBar], the widget that adds a platform menu bar to an
  ///    application.
  ///  * [PlatformMenu], the class that describes a menu item with children
  ///    that appear in a cascading menu.
  ///  * [PlatformMenuItem], the class that describes the leaves of a menu
  ///    hierarchy.
  void setMenus(List<PlatformMenuItem> topLevelMenus);

  /// Clears any existing platform-rendered menus and leaves the application
  /// with no menus.
  ///
  /// It is not necessary to call this before updating the menu with [setMenus].
  void clearMenus();

  /// This is called by [PlatformMenuBar] when it is initialized, to be sure that
  /// only one is active at a time.
  ///
  /// The [debugLockDelegate] function should be called before the first call to
  /// [setMenus].
  ///
  /// If the lock is successfully acquired, [debugLockDelegate] will return
  /// true.
  ///
  /// If your implementation of a [PlatformMenuDelegate] can have only limited
  /// active instances, enforce it when you override this function.
  ///
  /// See also:
  ///
  ///  * [debugUnlockDelegate], where the delegate is unlocked.
  bool debugLockDelegate(BuildContext context);

  /// This is called by [PlatformMenuBar] when it is disposed, so that another
  /// one can take over.
  ///
  /// If the [debugUnlockDelegate] successfully unlocks the delegate, it will
  /// return true.
  ///
  /// See also:
  ///
  ///  * [debugLockDelegate], where the delegate is locked.
  bool debugUnlockDelegate(BuildContext context);
}

/// The signature for a function that generates unique menu item IDs for
/// serialization of a [PlatformMenuItem].
typedef MenuItemSerializableIdGenerator = int Function(PlatformMenuItem item);

/// The platform menu delegate that handles the built-in macOS platform menu
/// generation using the 'flutter/menu' channel.
///
/// An instance of this class is set on [WidgetsBinding.platformMenuDelegate] by
/// default when the [WidgetsBinding] is initialized.
///
/// See also:
///
///  * [PlatformMenuBar], the widget that adds a platform menu bar to an
///    application.
///  * [PlatformMenu], the class that describes a menu item with children
///    that appear in a cascading menu.
///  * [PlatformMenuItem], the class that describes the leaves of a menu
///    hierarchy.
class DefaultPlatformMenuDelegate extends PlatformMenuDelegate {
  /// Creates a const [DefaultPlatformMenuDelegate].
  ///
  /// The optional [channel] argument defines the channel used to communicate
  /// with the platform. It defaults to [SystemChannels.menu] if not supplied.
  DefaultPlatformMenuDelegate({MethodChannel? channel})
    : channel = channel ?? SystemChannels.menu,
      _idMap = <int, PlatformMenuItem>{} {
    this.channel.setMethodCallHandler(_methodCallHandler);
  }

  // Map of distributed IDs to menu items.
  final Map<int, PlatformMenuItem> _idMap;
  // An ever increasing value used to dole out IDs.
  int _serial = 0;
  // The context used to "lock" this delegate to a specific instance of
  // PlatformMenuBar to make sure there is only one.
  BuildContext? _lockedContext;

  @override
  void clearMenus() => setMenus(<PlatformMenuItem>[]);

  @override
  void setMenus(List<PlatformMenuItem> topLevelMenus) {
    _idMap.clear();
    final List<Map<String, Object?>> representation = <Map<String, Object?>>[];
    if (topLevelMenus.isNotEmpty) {
      for (final PlatformMenuItem childItem in topLevelMenus) {
        representation.addAll(childItem.toChannelRepresentation(this, getId: _getId));
      }
    }
    // Currently there's only ever one window, but the channel's format allows
    // more than one window's menu hierarchy to be defined.
    final Map<String, Object?> windowMenu = <String, Object?>{'0': representation};
    channel.invokeMethod<void>(_kMenuSetMethod, windowMenu);
  }

  /// Defines the channel that the [DefaultPlatformMenuDelegate] uses to
  /// communicate with the platform.
  ///
  /// Defaults to [SystemChannels.menu].
  final MethodChannel channel;

  /// Get the next serialization ID.
  ///
  /// This is called by each DefaultPlatformMenuDelegateSerializer when
  /// serializing a new object so that it has a unique ID.
  int _getId(PlatformMenuItem item) {
    _serial += 1;
    _idMap[_serial] = item;
    return _serial;
  }

  @override
  bool debugLockDelegate(BuildContext context) {
    assert(() {
      // It's OK to lock if the lock isn't set, but not OK if a different
      // context is locking it.
      if (_lockedContext != null && _lockedContext != context) {
        return false;
      }
      _lockedContext = context;
      return true;
    }());
    return true;
  }

  @override
  bool debugUnlockDelegate(BuildContext context) {
    assert(() {
      // It's OK to unlock if the lock isn't set, but not OK if a different
      // context is unlocking it.
      if (_lockedContext != null && _lockedContext != context) {
        return false;
      }
      _lockedContext = null;
      return true;
    }());
    return true;
  }

  // Handles the method calls from the plugin to forward to selection and
  // open/close callbacks.
  Future<void> _methodCallHandler(MethodCall call) async {
    final int id = call.arguments as int;
    assert(
      _idMap.containsKey(id),
      'Received a menu ${call.method} for a menu item with an ID that was not recognized: $id',
    );
    if (!_idMap.containsKey(id)) {
      return;
    }
    final PlatformMenuItem item = _idMap[id]!;
    if (call.method == _kMenuSelectedCallbackMethod) {
      assert(
        item.onSelected == null || item.onSelectedIntent == null,
        'Only one of PlatformMenuItem.onSelected or PlatformMenuItem.onSelectedIntent may be specified',
      );
      item.onSelected?.call();
      if (item.onSelectedIntent != null) {
        Actions.maybeInvoke(FocusManager.instance.primaryFocus!.context!, item.onSelectedIntent!);
      }
    } else if (call.method == _kMenuItemOpenedMethod) {
      item.onOpen?.call();
    } else if (call.method == _kMenuItemClosedMethod) {
      item.onClose?.call();
    }
  }
}

/// A menu bar that uses the platform's native APIs to construct and render a
/// menu described by a [PlatformMenu]/[PlatformMenuItem] hierarchy.
///
/// This widget is especially useful on macOS, where a system menu is a required
/// part of every application. Flutter only includes support for macOS out of
/// the box, but support for other platforms may be provided via plugins that
/// set [WidgetsBinding.platformMenuDelegate] in their initialization.
///
/// The [menus] member contains [PlatformMenuItem]s, which configure the
/// properties of the menus on the platform menu bar.
///
/// As far as Flutter is concerned, this widget has no visual representation,
/// and intercepts no events: it just returns the [child] from its build
/// function. This is because all of the rendering, shortcuts, and event
/// handling for the menu is handled by the plugin on the host platform. It is
/// only part of the widget tree to provide a convenient refresh mechanism for
/// the menu data.
///
/// There can only be one [PlatformMenuBar] at a time using the same
/// [PlatformMenuDelegate]. It will assert if more than one is detected.
///
/// When calling [toStringDeep] on this widget, it will give a tree of
/// [PlatformMenuItem]s, not a tree of widgets.
///
/// {@tool sample} This example shows a [PlatformMenuBar] that contains a single
/// top level menu, containing three items for "About", a toggleable menu item
/// for showing a message, a cascading submenu with message choices, and "Quit".
///
/// **This example will only work on macOS.**
///
/// ** See code in examples/api/lib/material/platform_menu_bar/platform_menu_bar.0.dart **
/// {@end-tool}
///
/// The menus could just as effectively be managed without using the widget tree
/// by using the following code, but mixing this usage with [PlatformMenuBar] is
/// not recommended, since it will overwrite the menu configuration when it is
/// rebuilt:
///
/// ```dart
/// List<PlatformMenuItem> menus = <PlatformMenuItem>[ /* Define menus... */ ];
/// WidgetsBinding.instance.platformMenuDelegate.setMenus(menus);
/// ```
class PlatformMenuBar extends StatefulWidget with DiagnosticableTreeMixin {
  /// Creates a const [PlatformMenuBar].
  ///
  /// The [child] and [menus] attributes are required.
  const PlatformMenuBar({super.key, required this.menus, this.child});

  /// The widget below this widget in the tree.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget? child;

  /// The list of menu items that are the top level children of the
  /// [PlatformMenuBar].
  ///
  /// The [menus] member contains [PlatformMenuItem]s. They will not be part of
  /// the widget tree, since they are not widgets. They are provided to
  /// configure the properties of the menus on the platform menu bar.
  ///
  /// Also, a Widget in Flutter is immutable, so directly modifying the
  /// [menus] with `List` APIs such as
  /// `somePlatformMenuBarWidget.menus.add(...)` will result in incorrect
  /// behaviors. Whenever the menus list is modified, a new list object
  /// should be provided.
  final List<PlatformMenuItem> menus;

  @override
  State<PlatformMenuBar> createState() => _PlatformMenuBarState();

  @override
  List<DiagnosticsNode> debugDescribeChildren() {
    return menus
        .map<DiagnosticsNode>((PlatformMenuItem child) => child.toDiagnosticsNode())
        .toList();
  }
}

class _PlatformMenuBarState extends State<PlatformMenuBar> {
  List<PlatformMenuItem> descendants = <PlatformMenuItem>[];

  @override
  void initState() {
    super.initState();
    assert(
      WidgetsBinding.instance.platformMenuDelegate.debugLockDelegate(context),
      'More than one active $PlatformMenuBar detected. Only one active '
      'platform-rendered menu bar is allowed at a time.',
    );
    WidgetsBinding.instance.platformMenuDelegate.clearMenus();
    _updateMenu();
  }

  @override
  void dispose() {
    assert(
      WidgetsBinding.instance.platformMenuDelegate.debugUnlockDelegate(context),
      'tried to unlock the $DefaultPlatformMenuDelegate more than once with context $context.',
    );
    WidgetsBinding.instance.platformMenuDelegate.clearMenus();
    super.dispose();
  }

  @override
  void didUpdateWidget(PlatformMenuBar oldWidget) {
    super.didUpdateWidget(oldWidget);
    final List<PlatformMenuItem> newDescendants = <PlatformMenuItem>[
      for (final PlatformMenuItem item in widget.menus) ...<PlatformMenuItem>[
        item,
        ...item.descendants,
      ],
    ];
    if (!listEquals(newDescendants, descendants)) {
      descendants = newDescendants;
      _updateMenu();
    }
  }

  // Updates the data structures for the menu and send them to the platform
  // plugin.
  void _updateMenu() {
    WidgetsBinding.instance.platformMenuDelegate.setMenus(widget.menus);
  }

  @override
  Widget build(BuildContext context) {
    // PlatformMenuBar is really about managing the platform menu bar, and
    // doesn't do any rendering or event handling in Flutter.
    return widget.child ?? const SizedBox();
  }
}

/// A class for representing menu items that have child submenus.
///
/// See also:
///
///  * [PlatformMenuItem], a class representing a leaf menu item in a
///    [PlatformMenuBar].
class PlatformMenu extends PlatformMenuItem with DiagnosticableTreeMixin {
  /// Creates a const [PlatformMenu].
  ///
  /// The [label] and [menus] fields are required.
  const PlatformMenu({required super.label, this.onOpen, this.onClose, required this.menus});

  @override
  final VoidCallback? onOpen;

  @override
  final VoidCallback? onClose;

  /// The menu items in the submenu opened by this menu item.
  ///
  /// If this is an empty list, this [PlatformMenu] will be disabled.
  final List<PlatformMenuItem> menus;

  /// Returns all descendant [PlatformMenuItem]s of this item.
  @override
  List<PlatformMenuItem> get descendants => getDescendants(this);

  /// Returns all descendants of the given item.
  ///
  /// This API is supplied so that implementers of [PlatformMenu] can share
  /// this implementation.
  static List<PlatformMenuItem> getDescendants(PlatformMenu item) {
    return <PlatformMenuItem>[
      for (final PlatformMenuItem child in item.menus) ...<PlatformMenuItem>[
        child,
        ...child.descendants,
      ],
    ];
  }

  @override
  Iterable<Map<String, Object?>> toChannelRepresentation(
    PlatformMenuDelegate delegate, {
    required MenuItemSerializableIdGenerator getId,
  }) {
    return <Map<String, Object?>>[serialize(this, delegate, getId)];
  }

  /// Converts the supplied object to the correct channel representation for the
  /// 'flutter/menu' channel.
  ///
  /// This API is supplied so that implementers of [PlatformMenu] can share
  /// this implementation.
  static Map<String, Object?> serialize(
    PlatformMenu item,
    PlatformMenuDelegate delegate,
    MenuItemSerializableIdGenerator getId,
  ) {
    final List<Map<String, Object?>> result = <Map<String, Object?>>[];
    for (final PlatformMenuItem childItem in item.menus) {
      result.addAll(childItem.toChannelRepresentation(delegate, getId: getId));
    }
    // To avoid doing type checking for groups, just filter out when there are
    // multiple sequential dividers, or when they are first or last, since
    // groups may be interleaved with non-groups, and non-groups may also add
    // dividers.
    Map<String, Object?>? previousItem;
    result.removeWhere((Map<String, Object?> item) {
      if (previousItem == null && item[_kIsDividerKey] == true) {
        // Strip any leading dividers.
        return true;
      }
      if (previousItem != null &&
          previousItem![_kIsDividerKey] == true &&
          item[_kIsDividerKey] == true) {
        // Strip any duplicate dividers.
        return true;
      }
      previousItem = item;
      return false;
    });
    if (result.lastOrNull case {_kIsDividerKey: true}) {
      result.removeLast();
    }
    return <String, Object?>{
      _kIdKey: getId(item),
      _kLabelKey: item.label,
      _kEnabledKey: item.menus.isNotEmpty,
      _kChildrenKey: result,
    };
  }

  @override
  List<DiagnosticsNode> debugDescribeChildren() {
    return menus
        .map<DiagnosticsNode>((PlatformMenuItem child) => child.toDiagnosticsNode())
        .toList();
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(StringProperty('label', label));
    properties.add(FlagProperty('enabled', value: menus.isNotEmpty, ifFalse: 'DISABLED'));
  }
}

/// A class that groups other menu items into sections delineated by dividers.
///
/// Visual dividers will be added before and after this group if other menu
/// items appear in the [PlatformMenu], and the leading one omitted if it is
/// first and the trailing one omitted if it is last in the menu.
class PlatformMenuItemGroup extends PlatformMenuItem {
  /// Creates a const [PlatformMenuItemGroup].
  ///
  /// The [members] field is required.
  const PlatformMenuItemGroup({required this.members}) : super(label: '');

  /// The [PlatformMenuItem]s that are members of this menu item group.
  ///
  /// An assertion will be thrown if there isn't at least one member of the group.
  @override
  final List<PlatformMenuItem> members;

  @override
  Iterable<Map<String, Object?>> toChannelRepresentation(
    PlatformMenuDelegate delegate, {
    required MenuItemSerializableIdGenerator getId,
  }) {
    assert(members.isNotEmpty, 'There must be at least one member in a PlatformMenuItemGroup');
    return serialize(this, delegate, getId: getId);
  }

  /// Converts the supplied object to the correct channel representation for the
  /// 'flutter/menu' channel.
  ///
  /// This API is supplied so that implementers of [PlatformMenuItemGroup] can share
  /// this implementation.
  static Iterable<Map<String, Object?>> serialize(
    PlatformMenuItem group,
    PlatformMenuDelegate delegate, {
    required MenuItemSerializableIdGenerator getId,
  }) {
    return <Map<String, Object?>>[
      <String, Object?>{_kIdKey: getId(group), _kIsDividerKey: true},
      for (final PlatformMenuItem item in group.members)
        ...item.toChannelRepresentation(delegate, getId: getId),
      <String, Object?>{_kIdKey: getId(group), _kIsDividerKey: true},
    ];
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(IterableProperty<PlatformMenuItem>('members', members));
  }
}

/// A class for [PlatformMenuItem]s that do not have submenus (as a [PlatformMenu]
/// would), but can be selected.
///
/// These [PlatformMenuItem]s are the leaves of the menu item tree, and [onSelected]
/// will be called when they are selected by clicking on them, or via an
/// optional keyboard [shortcut].
///
/// See also:
///
///  * [PlatformMenu], a menu item that opens a submenu.
class PlatformMenuItem with Diagnosticable {
  /// Creates a const [PlatformMenuItem].
  ///
  /// The [label] attribute is required.
  const PlatformMenuItem({
    required this.label,
    this.shortcut,
    this.onSelected,
    this.onSelectedIntent,
  }) : assert(
         onSelected == null || onSelectedIntent == null,
         'Only one of onSelected or onSelectedIntent may be specified',
       );

  /// The required label used for rendering the menu item.
  final String label;

  /// The optional shortcut that selects this [PlatformMenuItem].
  ///
  /// This shortcut is only enabled when [onSelected] is set.
  final MenuSerializableShortcut? shortcut;

  /// An optional callback that is called when this [PlatformMenuItem] is
  /// selected.
  ///
  /// At most one of [onSelected] and [onSelectedIntent] may be set. If neither
  /// field is set, this menu item will be disabled.
  final VoidCallback? onSelected;

  /// Returns a callback, if any, to be invoked if the platform menu receives a
  /// "Menu.opened" method call from the platform for this item.
  ///
  /// Only items that have submenus will have this callback invoked.
  ///
  /// The default implementation returns null.
  VoidCallback? get onOpen => null;

  /// Returns a callback, if any, to be invoked if the platform menu receives a
  /// "Menu.closed" method call from the platform for this item.
  ///
  /// Only items that have submenus will have this callback invoked.
  ///
  /// The default implementation returns null.
  VoidCallback? get onClose => null;

  /// An optional intent that is invoked when this [PlatformMenuItem] is
  /// selected.
  ///
  /// At most one of [onSelected] and [onSelectedIntent] may be set. If neither
  /// field is set, this menu item will be disabled.
  final Intent? onSelectedIntent;

  /// Returns all descendant [PlatformMenuItem]s of this item.
  ///
  /// Returns an empty list if this type of menu item doesn't have
  /// descendants.
  List<PlatformMenuItem> get descendants => const <PlatformMenuItem>[];

  /// Returns the list of group members if this menu item is a "grouping" menu
  /// item, such as [PlatformMenuItemGroup].
  ///
  /// Defaults to an empty list.
  List<PlatformMenuItem> get members => const <PlatformMenuItem>[];

  /// Converts the representation of this item into a map suitable for sending
  /// over the default "flutter/menu" channel used by [DefaultPlatformMenuDelegate].
  ///
  /// The `delegate` is the [PlatformMenuDelegate] that is requesting the
  /// serialization.
  ///
  /// The `getId` parameter is a [MenuItemSerializableIdGenerator] function that
  /// generates a unique ID for each menu item, which is to be returned in the
  /// "id" field of the menu item data.
  Iterable<Map<String, Object?>> toChannelRepresentation(
    PlatformMenuDelegate delegate, {
    required MenuItemSerializableIdGenerator getId,
  }) {
    return <Map<String, Object?>>[PlatformMenuItem.serialize(this, delegate, getId)];
  }

  /// Converts the given [PlatformMenuItem] into a data structure accepted by
  /// the 'flutter/menu' method channel method 'Menu.SetMenu'.
  ///
  /// This API is supplied so that implementers of [PlatformMenuItem] can share
  /// this implementation.
  static Map<String, Object?> serialize(
    PlatformMenuItem item,
    PlatformMenuDelegate delegate,
    MenuItemSerializableIdGenerator getId,
  ) {
    final MenuSerializableShortcut? shortcut = item.shortcut;
    return <String, Object?>{
      _kIdKey: getId(item),
      _kLabelKey: item.label,
      _kEnabledKey: item.onSelected != null || item.onSelectedIntent != null,
      if (shortcut != null) ...shortcut.serializeForMenu().toChannelRepresentation(),
    };
  }

  @override
  String toStringShort() => '${describeIdentity(this)}($label)';

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(StringProperty('label', label));
    properties.add(
      DiagnosticsProperty<MenuSerializableShortcut?>('shortcut', shortcut, defaultValue: null),
    );
    properties.add(FlagProperty('enabled', value: onSelected != null, ifFalse: 'DISABLED'));
  }
}

/// A class that represents a menu item that is provided by the platform.
///
/// This is used to add things like the "About" and "Quit" menu items to a
/// platform menu.
///
/// The [type] enum determines which type of platform defined menu will be
/// added.
///
/// This is most useful on a macOS platform where there are many different types
/// of platform provided menu items in the standard menu setup.
///
/// In order to know if a [PlatformProvidedMenuItem] is available on a
/// particular platform, call [PlatformProvidedMenuItem.hasMenu].
///
/// If the platform does not support the given [type], then the menu item will
/// throw an [ArgumentError] when it is sent to the platform.
///
/// See also:
///
///  * [PlatformMenuBar] which takes these items for inclusion in a
///    platform-rendered menu bar.
class PlatformProvidedMenuItem extends PlatformMenuItem {
  /// Creates a const [PlatformProvidedMenuItem] of the appropriate type. Throws if the
  /// platform doesn't support the given default menu type.
  ///
  /// The [type] argument is required.
  const PlatformProvidedMenuItem({required this.type, this.enabled = true})
    : super(label: ''); // The label is ignored for platform provided menus.

  /// The type of default menu this is.
  ///
  /// See [PlatformProvidedMenuItemType] for the different types available. Not
  /// all of the types will be available on every platform. Use [hasMenu] to
  /// determine if the current platform has a given default menu item.
  ///
  /// If the platform does not support the given [type], then the menu item will
  /// throw an [ArgumentError] in debug mode.
  final PlatformProvidedMenuItemType type;

  /// True if this [PlatformProvidedMenuItem] should be enabled or not.
  final bool enabled;

  /// Checks to see if the given default menu type is supported on this
  /// platform.
  static bool hasMenu(PlatformProvidedMenuItemType menu) {
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
      case TargetPlatform.iOS:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        return false;
      case TargetPlatform.macOS:
        return const <PlatformProvidedMenuItemType>{
          PlatformProvidedMenuItemType.about,
          PlatformProvidedMenuItemType.quit,
          PlatformProvidedMenuItemType.servicesSubmenu,
          PlatformProvidedMenuItemType.hide,
          PlatformProvidedMenuItemType.hideOtherApplications,
          PlatformProvidedMenuItemType.showAllApplications,
          PlatformProvidedMenuItemType.startSpeaking,
          PlatformProvidedMenuItemType.stopSpeaking,
          PlatformProvidedMenuItemType.toggleFullScreen,
          PlatformProvidedMenuItemType.minimizeWindow,
          PlatformProvidedMenuItemType.zoomWindow,
          PlatformProvidedMenuItemType.arrangeWindowsInFront,
        }.contains(menu);
    }
  }

  @override
  Iterable<Map<String, Object?>> toChannelRepresentation(
    PlatformMenuDelegate delegate, {
    required MenuItemSerializableIdGenerator getId,
  }) {
    assert(() {
      if (!hasMenu(type)) {
        throw ArgumentError(
          'Platform ${defaultTargetPlatform.name} has no platform provided menu for '
          '$type. Call PlatformProvidedMenuItem.hasMenu to determine this before '
          'instantiating one.',
        );
      }
      return true;
    }());

    return <Map<String, Object?>>[
      <String, Object?>{
        _kIdKey: getId(this),
        _kEnabledKey: enabled,
        _kPlatformDefaultMenuKey: type.index,
      },
    ];
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(FlagProperty('enabled', value: enabled, ifFalse: 'DISABLED'));
  }
}

/// The list of possible platform provided, prebuilt menus for use in a
/// [PlatformMenuBar].
///
/// These are menus that the platform typically provides that cannot be
/// reproduced in Flutter without calling platform functions, but are standard
/// on the platform.
///
/// Examples include things like the "Quit" or "Services" menu items on macOS.
/// Not all platforms support all menu item types. Use
/// [PlatformProvidedMenuItem.hasMenu] to know if a particular type is supported
/// on a the current platform.
///
/// Add these to your [PlatformMenuBar] using the [PlatformProvidedMenuItem]
/// class.
///
/// You can tell if the platform provides the given menu using the
/// [PlatformProvidedMenuItem.hasMenu] method.
// Must be kept in sync with the plugin code's enum of the same name.
enum PlatformProvidedMenuItemType {
  /// The system provided "About" menu item.
  ///
  /// On macOS, this is the `orderFrontStandardAboutPanel` default menu.
  about,

  /// The system provided "Quit" menu item.
  ///
  /// On macOS, this is the `terminate` default menu.
  ///
  /// This menu item will exit the application when activated.
  quit,

  /// The system provided "Services" submenu.
  ///
  /// This submenu provides a list of system provided application services.
  ///
  /// This default menu is only supported on macOS.
  servicesSubmenu,

  /// The system provided "Hide" menu item.
  ///
  /// This menu item hides the application window.
  ///
  /// On macOS, this is the `hide` default menu.
  ///
  /// This default menu is only supported on macOS.
  hide,

  /// The system provided "Hide Others" menu item.
  ///
  /// This menu item hides other application windows.
  ///
  /// On macOS, this is the `hideOtherApplications` default menu.
  ///
  /// This default menu is only supported on macOS.
  hideOtherApplications,

  /// The system provided "Show All" menu item.
  ///
  /// This menu item shows all hidden application windows.
  ///
  /// On macOS, this is the `unhideAllApplications` default menu.
  ///
  /// This default menu is only supported on macOS.
  showAllApplications,

  /// The system provided "Start Dictation..." menu item.
  ///
  /// This menu item tells the system to start the screen reader.
  ///
  /// On macOS, this is the `startSpeaking` default menu.
  ///
  /// This default menu is currently only supported on macOS.
  startSpeaking,

  /// The system provided "Stop Dictation..." menu item.
  ///
  /// This menu item tells the system to stop the screen reader.
  ///
  /// On macOS, this is the `stopSpeaking` default menu.
  ///
  /// This default menu is currently only supported on macOS.
  stopSpeaking,

  /// The system provided "Enter Full Screen" menu item.
  ///
  /// This menu item tells the system to toggle full screen mode for the window.
  ///
  /// On macOS, this is the `toggleFullScreen` default menu.
  ///
  /// This default menu is currently only supported on macOS.
  toggleFullScreen,

  /// The system provided "Minimize" menu item.
  ///
  /// This menu item tells the system to minimize the window.
  ///
  /// On macOS, this is the `performMiniaturize` default menu.
  ///
  /// This default menu is currently only supported on macOS.
  minimizeWindow,

  /// The system provided "Zoom" menu item.
  ///
  /// This menu item tells the system to expand the window size.
  ///
  /// On macOS, this is the `performZoom` default menu.
  ///
  /// This default menu is currently only supported on macOS.
  zoomWindow,

  /// The system provided "Bring To Front" menu item.
  ///
  /// This menu item tells the system to stack the window above other windows.
  ///
  /// On macOS, this is the `arrangeInFront` default menu.
  ///
  /// This default menu is currently only supported on macOS.
  arrangeWindowsInFront,
}
