// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';

import 'basic.dart';
import 'editable_text.dart';
import 'framework.dart';
import 'localizations.dart';
import 'media_query.dart';
import 'text_selection_toolbar_anchors.dart';

/// Displays the system context menu on top of the Flutter view.
///
/// Currently, only supports iOS 16.0 and above and displays nothing on other
/// platforms.
///
/// The context menu is the menu that appears, for example, when doing text
/// selection. Flutter typically draws this menu itself, but this class deals
/// with the platform-rendered context menu instead.
///
/// There can only be one system context menu visible at a time. Building this
/// widget when the system context menu is already visible will hide the old one
/// and display this one. A system context menu that is hidden is informed via
/// [onSystemHide].
///
/// To check if the current device supports showing the system context menu,
/// call [isSupported].
///
/// {@tool dartpad}
/// This example shows how to create a [TextField] that uses the system context
/// menu where supported and does not show a system notification when the user
/// presses the "Paste" button.
///
/// ** See code in examples/api/lib/widgets/system_context_menu/system_context_menu.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [SystemContextMenuController], which directly controls the hiding and
///    showing of the system context menu.
class SystemContextMenu extends StatefulWidget {
  /// Creates an instance of [SystemContextMenu] that points to the given
  /// [anchor].
  SystemContextMenu._({super.key, required this.anchor, this.items, this.onSystemHide})
    : assert(items == null || !_containsDuplicates(items));

  /// Creates an instance of [SystemContextMenu] for the field indicated by the
  /// given [EditableTextState].
  factory SystemContextMenu.editableText({
    Key? key,
    required EditableTextState editableTextState,
    List<SystemContextMenuItem>? items,
  }) {
    final (startGlyphHeight: double startGlyphHeight, endGlyphHeight: double endGlyphHeight) =
        editableTextState.getGlyphHeights();

    return SystemContextMenu._(
      key: key,
      anchor: TextSelectionToolbarAnchors.getSelectionRect(
        editableTextState.renderEditable,
        startGlyphHeight,
        endGlyphHeight,
        editableTextState.renderEditable.getEndpointsForSelection(
          editableTextState.textEditingValue.selection,
        ),
      ),
      items: items,
      onSystemHide: editableTextState.hideToolbar,
    );
  }

  /// The [Rect] that the context menu should point to.
  final Rect anchor;

  /// A list of the items to be displayed in the system context menu.
  ///
  /// If none are given, the items will be inferred by the platform based on the
  /// current [TextInputConnection].
  ///
  /// Built-in items will only be shown when relevant. For example, if
  /// [SystemContextMenuItem.copy] is passed, the copy button will only be shown
  /// when there is a non-empty selection and not when the selection is
  /// collapsed. It's not necessary to manually add and remove these items based
  /// on the state of the field.
  final List<SystemContextMenuItem>? items;

  /// Called when the system hides this context menu.
  ///
  /// For example, tapping outside of the context menu typically causes the
  /// system to hide the menu.
  ///
  /// This is not called when showing a new system context menu causes another
  /// to be hidden.
  final VoidCallback? onSystemHide;

  /// Whether the current device supports showing the system context menu.
  ///
  /// Currently, this is only supported on newer versions of iOS.
  static bool isSupported(BuildContext context) {
    return MediaQuery.maybeSupportsShowingSystemContextMenu(context) ?? false;
  }

  /// Returns true only if the given list contains at least one pair of
  /// identical SystemContextMenuItems.
  static bool _containsDuplicates(List<SystemContextMenuItem> items) {
    final Set<SystemContextMenuItem> uniqueItems = <SystemContextMenuItem>{};

    for (final SystemContextMenuItem item in items) {
      if (uniqueItems.contains(item)) {
        return true;
      }
      uniqueItems.add(item);
    }

    return false;
  }

  @override
  State<SystemContextMenu> createState() => _SystemContextMenuState();
}

class _SystemContextMenuState extends State<SystemContextMenu> {
  late final SystemContextMenuController _systemContextMenuController;

  /// Return the SystemContextMenuItemData for the given SystemContextMenuItem.
  ///
  /// SystemContextMenuItem is a format that is designed to be consumed as
  /// SystemContextMenu.items, where users might want a default localized title
  /// to be set for them.
  ///
  /// SystemContextMenuItemData is a format that is meant to be consumed by
  /// SystemContextMenuController.show, where there is no expectation that
  /// localizations can be used under the hood.
  SystemContextMenuItemData _itemToData(
    SystemContextMenuItem item,
    WidgetsLocalizations localizations,
  ) {
    return switch (item) {
      SystemContextMenuItemCut() => const SystemContextMenuItemDataCut(),
      SystemContextMenuItemCopy() => const SystemContextMenuItemDataCopy(),
      SystemContextMenuItemPaste() => const SystemContextMenuItemDataPaste(),
      SystemContextMenuItemSelectAll() => const SystemContextMenuItemDataSelectAll(),
      SystemContextMenuItemLookUp() => SystemContextMenuItemDataLookUp(
        title: item.title ?? localizations.lookUpButtonLabel,
      ),
      SystemContextMenuItemSearchWeb() => SystemContextMenuItemDataSearchWeb(
        title: item.title ?? localizations.searchWebButtonLabel,
      ),
      SystemContextMenuItemShare() => SystemContextMenuItemDataShare(
        title: item.title ?? localizations.shareButtonLabel,
      ),
      SystemContextMenuItemCustom() => SystemContextMenuItemDataCustom(
        title: item.title!,
        onPressed: item.onPressed!,
      ),
    };
  }

  bool _include(SystemContextMenuItem item, EditableTextState editableTextState) {
    return switch (item) {
      SystemContextMenuItemCopy() => editableTextState.copyEnabled,
      SystemContextMenuItemCut() => editableTextState.cutEnabled,
      SystemContextMenuItemPaste() => editableTextState.pasteEnabled,
      SystemContextMenuItemSelectAll() => editableTextState.selectAllEnabled,
      SystemContextMenuItemLookUp() => editableTextState.lookUpEnabled,
      SystemContextMenuItemSearchWeb() => editableTextState.searchWebEnabled,
      SystemContextMenuItemShare() => editableTextState.shareEnabled,
      SystemContextMenuItemCustom() => true,
    };
  }

  @override
  void initState() {
    super.initState();
    _systemContextMenuController = SystemContextMenuController(onSystemHide: widget.onSystemHide);
  }

  @override
  void dispose() {
    _systemContextMenuController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    assert(SystemContextMenu.isSupported(context));
    final WidgetsLocalizations localizations = WidgetsLocalizations.of(context);
    final Iterable<SystemContextMenuItemData>? datas = widget.items?.map(
      (SystemContextMenuItem item) => _itemToData(item, localizations),
    );
    _systemContextMenuController.show(widget.anchor, datas?.toList());

    return const SizedBox.shrink();
  }
}

// TODO(justinmc): Still need to decide to name this as ios specific or not.
// Probably should make it ios-specific, because these classes encode
// ios-specific logic, such as the fact that you can't change the title of the
// paste button, etc.
/// Describes a context menu button that will be rendered in the system context
/// menu and not by Flutter itself.
///
/// See also:
///
///  * [SystemContextMenu], a widget that can be used to display the system
///    context menu.
///  * [SystemContextMenuItemData], which performs a similar role but at the
///    method channel level and mirrors the requirements of the method channel
///    API.
///  * [ContextMenuButtonItem], which performs a similar role for Flutter-drawn
///    context menus.
@immutable
sealed class SystemContextMenuItem {
  const SystemContextMenuItem();

  /// The text to display to the user.
  ///
  /// Not exposed for some built-in menu items whose title is always set by the
  /// platform.
  String? get title => null;

  /// The callback to be called when the menu item is pressed.
  ///
  /// Not exposed for built-in menu items, which handle their own action when
  /// pressed.
  VoidCallback? get onPressed => null;

  @override
  int get hashCode => Object.hash(title, onPressed);

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is SystemContextMenuItem && other.title == title && other.onPressed == onPressed;
  }
}

/// Creates an instance of [SystemContextMenuItem] for the system's built-in cut
/// button.
///
/// Should only appear when there is a selection that can be cut.
///
/// The title and action are both handled by the platform.
///
/// See also:
///
///  * [SystemContextMenu], a widget that can be used to display the system
///    context menu.
///  * [SystemContextMenuItemDataCut], which specifies the data to be sent to
///    the platform for this same button.
class SystemContextMenuItemCut extends SystemContextMenuItem {
  /// Creates an instance of [SystemContextMenuItemCut].
  const SystemContextMenuItemCut();
}

/// Creates an instance of [SystemContextMenuItem] for the system's built-in
/// copy button.
///
/// Should only appear when there is a selection that can be copied.
///
/// The title and action are both handled by the platform.
///
/// See also:
///
///  * [SystemContextMenu], a widget that can be used to display the system
///    context menu.
///  * [SystemContextMenuItemDataCopy], which specifies the data to be sent to
///    the platform for this same button.
class SystemContextMenuItemCopy extends SystemContextMenuItem {
  /// Creates an instance of [SystemContextMenuItemCopy].
  const SystemContextMenuItemCopy();
}

/// Creates an instance of [SystemContextMenuItem] for the system's built-in
/// paste button.
///
/// Should only appear when the field can receive pasted content.
///
/// The title and action are both handled by the platform.
///
/// See also:
///
///  * [SystemContextMenu], a widget that can be used to display the system
///    context menu.
///  * [SystemContextMenuItemDataPaste], which specifies the data to be sent to
///    the platform for this same button.
class SystemContextMenuItemPaste extends SystemContextMenuItem {
  /// Creates an instance of [SystemContextMenuItemPaste].
  const SystemContextMenuItemPaste();
}

/// Creates an instance of [SystemContextMenuItem] for the system's built-in
/// select all button.
///
/// Should only appear when the field can have its selection changed.
///
/// The title and action are both handled by the platform.
///
/// See also:
///
///  * [SystemContextMenu], a widget that can be used to display the system
///    context menu.
///  * [SystemContextMenuItemDataSelectAll], which specifies the data to be sent
///    to the platform for this same button.
class SystemContextMenuItemSelectAll extends SystemContextMenuItem {
  /// Creates an instance of [SystemContextMenuItemSelectAll].
  const SystemContextMenuItemSelectAll();
}

/// Creates an instance of [SystemContextMenuItem] for the
/// system's built-in look up button.
///
/// Should only appear when content is selected.
///
/// The [title] is optional, but it must be specified before being sent to the
/// platform. Typically it should be set to
/// [WidgetsLocalizations.lookUpButtonLabel].
///
/// The action is handled by the platform.
///
/// See also:
///
///  * [SystemContextMenu], a widget that can be used to display the system
///    context menu.
///  * [SystemContextMenuItemDataLookup], which specifies the data to be sent to
///    the platform for this same button.
class SystemContextMenuItemLookUp extends SystemContextMenuItem {
  /// Creates an instance of [SystemContextMenuItemLookUp].
  const SystemContextMenuItemLookUp({this.title});

  @override
  final String? title;

  @override
  String toString() {
    return 'SystemContextMenuItemLookUp(title: $title)';
  }
}

/// Creates an instance of [SystemContextMenuItem] for the
/// system's built-in search web button.
///
/// Should only appear when content is selected.
///
/// The [title] is optional, but it must be specified before being sent to the
/// platform. Typically it should be set to
/// [WidgetsLocalizations.searchWebButtonlabel].
///
/// The action is handled by the platform.
///
/// See also:
///
///  * [SystemContextMenu], a widget that can be used to display the system
///    context menu.
///  * [SystemContextMenuItemDataSearchWeb], which specifies the data to be sent
///    to the platform for this same button.
class SystemContextMenuItemSearchWeb extends SystemContextMenuItem {
  /// Creates an instance of [SystemContextMenuItemSearchWeb].
  const SystemContextMenuItemSearchWeb({this.title});

  @override
  final String? title;

  @override
  String toString() {
    return 'SystemContextMenuItemSearchWeb(title: $title)';
  }
}

/// Creates an instance of [SystemContextMenuItem] for the
/// system's built-in share button.
///
/// Opens the system share dialog.
///
/// Should only appear when shareable content is selected.
///
/// The [title] is optional, but it must be specified before being sent to the
/// platform. Typically it should be set to
/// [WidgetsLocalizations.shareButtonLabel].
///
/// See also:
///
///  * [SystemContextMenu], a widget that can be used to display the system
///    context menu.
///  * [SystemContextMenuItemDataShare], which specifies the data to be sent to
///    the platform for this same button.
class SystemContextMenuItemShare extends SystemContextMenuItem {
  /// Creates an instance of [SystemContextMenuItemShare].
  const SystemContextMenuItemShare({this.title});

  @override
  final String? title;

  @override
  String toString() {
    return 'SystemContextMenuItemShare(title: $title)';
  }
}

// TODO(justinmc): Support the "custom" type.
// https://github.com/flutter/flutter/issues/103163
/// Creates an instance of [SystemContextMenuItem] for a custom menu item whose
/// [title] and [onPressed] are as specified.
///
/// See also:
///
///  * [SystemContextMenu], a widget that can be used to display the system
///    context menu.
///  * [SystemContextMenuItemDataCustom], which specifies the data to be sent
///    to the platform for this same button.
class SystemContextMenuItemCustom extends SystemContextMenuItem {
  /// Creates an instance of [SystemContextMenuItemCustom].
  const SystemContextMenuItemCustom({
    required String this.title,
    required VoidCallback this.onPressed,
  });

  @override
  final String? title;

  @override
  final VoidCallback? onPressed;

  @override
  String toString() {
    return 'SystemContextMenuItemCustom(title: $title, onPressed: $onPressed)';
  }
}
