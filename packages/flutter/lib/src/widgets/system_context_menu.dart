// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
library;

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
/// Pass [items] to specify the buttons that will appear in the menu. Any items
/// without a title will be given a default title from [WidgetsLocalizations].
///
/// By default, [items] will be set to the result of [getDefaultItems]. This
/// method considers the state of the [EditableTextState] so that, for example,
/// it will only include [IOSSystemContextMenuItemCopy] if there is currently a
/// selection to copy.
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
  const SystemContextMenu._({
    super.key,
    required this.anchor,
    required this.items,
    this.onSystemHide,
  });

  /// Creates an instance of [SystemContextMenu] for the field indicated by the
  /// given [EditableTextState].
  factory SystemContextMenu.editableText({
    Key? key,
    required EditableTextState editableTextState,
    List<IOSSystemContextMenuItem>? items,
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
      items: items ?? getDefaultItems(editableTextState),
      onSystemHide: editableTextState.hideToolbar,
    );
  }

  /// The [Rect] that the context menu should point to.
  final Rect anchor;

  /// A list of the items to be displayed in the system context menu.
  ///
  /// When passed, items will be shown regardless of the state of text input.
  /// For example, [IOSSystemContextMenuItemCopy] will produce a copy button
  /// even when there is no selection to copy. Use [EditableTextState] and/or
  /// the result of [getDefaultItems] to add and remove items based on the state
  /// of the input.
  ///
  /// Defaults to the result of [getDefaultItems].
  final List<IOSSystemContextMenuItem> items;

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

  /// The default [items] for the given [EditableTextState].
  ///
  /// For example, [IOSSystemContextMenuItemCopy] will only be included when the
  /// field represented by the [EditableTextState] has a selection.
  ///
  /// See also:
  ///
  ///  * [EditableTextState.contextMenuButtonItems], which provides the default
  ///    [ContextMenuButtonItem]s for the Flutter-rendered context menu.
  static List<IOSSystemContextMenuItem> getDefaultItems(EditableTextState editableTextState) {
    return <IOSSystemContextMenuItem>[
      if (editableTextState.copyEnabled) const IOSSystemContextMenuItemCopy(),
      if (editableTextState.cutEnabled) const IOSSystemContextMenuItemCut(),
      if (editableTextState.pasteEnabled) const IOSSystemContextMenuItemPaste(),
      if (editableTextState.selectAllEnabled) const IOSSystemContextMenuItemSelectAll(),
      if (editableTextState.lookUpEnabled) const IOSSystemContextMenuItemLookUp(),
      if (editableTextState.searchWebEnabled) const IOSSystemContextMenuItemSearchWeb(),
    ];
  }

  @override
  State<SystemContextMenu> createState() => _SystemContextMenuState();
}

class _SystemContextMenuState extends State<SystemContextMenu> {
  late final SystemContextMenuController _systemContextMenuController;

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

    if (widget.items.isNotEmpty) {
      final WidgetsLocalizations localizations = WidgetsLocalizations.of(context);
      final List<IOSSystemContextMenuItemData> itemDatas =
          widget.items.map((IOSSystemContextMenuItem item) => item.getData(localizations)).toList();
      _systemContextMenuController.showWithItems(widget.anchor, itemDatas);
    }

    return const SizedBox.shrink();
  }
}

/// Describes a context menu button that will be rendered in the iOS system
/// context menu and not by Flutter itself.
///
/// See also:
///
///  * [SystemContextMenu], a widget that can be used to display the system
///    context menu.
///  * [IOSSystemContextMenuItemData], which performs a similar role but at the
///    method channel level and mirrors the requirements of the method channel
///    API.
///  * [ContextMenuButtonItem], which performs a similar role for Flutter-drawn
///    context menus.
@immutable
sealed class IOSSystemContextMenuItem {
  const IOSSystemContextMenuItem();

  /// The text to display to the user.
  ///
  /// Not exposed for some built-in menu items whose title is always set by the
  /// platform.
  String? get title => null;

  /// Returns the representation of this class used by method channels.
  IOSSystemContextMenuItemData getData(WidgetsLocalizations localizations);

  @override
  int get hashCode => title.hashCode;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is IOSSystemContextMenuItem && other.title == title;
  }
}

/// Creates an instance of [IOSSystemContextMenuItem] for the system's built-in
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
///  * [IOSSystemContextMenuItemDataCopy], which specifies the data to be sent to
///    the platform for this same button.
final class IOSSystemContextMenuItemCopy extends IOSSystemContextMenuItem {
  /// Creates an instance of [IOSSystemContextMenuItemCopy].
  const IOSSystemContextMenuItemCopy();

  @override
  IOSSystemContextMenuItemDataCopy getData(WidgetsLocalizations localizations) {
    return const IOSSystemContextMenuItemDataCopy();
  }
}

/// Creates an instance of [IOSSystemContextMenuItem] for the system's built-in
/// cut button.
///
/// Should only appear when there is a selection that can be cut.
///
/// The title and action are both handled by the platform.
///
/// See also:
///
///  * [SystemContextMenu], a widget that can be used to display the system
///    context menu.
///  * [IOSSystemContextMenuItemDataCut], which specifies the data to be sent to
///    the platform for this same button.
final class IOSSystemContextMenuItemCut extends IOSSystemContextMenuItem {
  /// Creates an instance of [IOSSystemContextMenuItemCut].
  const IOSSystemContextMenuItemCut();

  @override
  IOSSystemContextMenuItemDataCut getData(WidgetsLocalizations localizations) {
    return const IOSSystemContextMenuItemDataCut();
  }
}

/// Creates an instance of [IOSSystemContextMenuItem] for the system's built-in
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
///  * [IOSSystemContextMenuItemDataPaste], which specifies the data to be sent
///     to the platform for this same button.
final class IOSSystemContextMenuItemPaste extends IOSSystemContextMenuItem {
  /// Creates an instance of [IOSSystemContextMenuItemPaste].
  const IOSSystemContextMenuItemPaste();

  @override
  IOSSystemContextMenuItemDataPaste getData(WidgetsLocalizations localizations) {
    return const IOSSystemContextMenuItemDataPaste();
  }
}

/// Creates an instance of [IOSSystemContextMenuItem] for the system's built-in
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
///  * [IOSSystemContextMenuItemDataSelectAll], which specifies the data to be
///     sent to the platform for this same button.
final class IOSSystemContextMenuItemSelectAll extends IOSSystemContextMenuItem {
  /// Creates an instance of [IOSSystemContextMenuItemSelectAll].
  const IOSSystemContextMenuItemSelectAll();

  @override
  IOSSystemContextMenuItemDataSelectAll getData(WidgetsLocalizations localizations) {
    return const IOSSystemContextMenuItemDataSelectAll();
  }
}

/// Creates an instance of [IOSSystemContextMenuItem] for the
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
///  * [IOSSystemContextMenuItemDataLookUp], which specifies the data to be sent
///    to the platform for this same button.
final class IOSSystemContextMenuItemLookUp extends IOSSystemContextMenuItem {
  /// Creates an instance of [IOSSystemContextMenuItemLookUp].
  const IOSSystemContextMenuItemLookUp({this.title});

  @override
  final String? title;

  @override
  IOSSystemContextMenuItemDataLookUp getData(WidgetsLocalizations localizations) {
    return IOSSystemContextMenuItemDataLookUp(title: title ?? localizations.lookUpButtonLabel);
  }

  @override
  String toString() {
    return 'IOSSystemContextMenuItemLookUp(title: $title)';
  }
}

/// Creates an instance of [IOSSystemContextMenuItem] for the
/// system's built-in search web button.
///
/// Should only appear when content is selected.
///
/// The [title] is optional, but it must be specified before being sent to the
/// platform. Typically it should be set to
/// [WidgetsLocalizations.searchWebButtonLabel].
///
/// The action is handled by the platform.
///
/// See also:
///
///  * [SystemContextMenu], a widget that can be used to display the system
///    context menu.
///  * [IOSSystemContextMenuItemDataSearchWeb], which specifies the data to be
///    sent to the platform for this same button.
final class IOSSystemContextMenuItemSearchWeb extends IOSSystemContextMenuItem {
  /// Creates an instance of [IOSSystemContextMenuItemSearchWeb].
  const IOSSystemContextMenuItemSearchWeb({this.title});

  @override
  final String? title;

  @override
  IOSSystemContextMenuItemDataSearchWeb getData(WidgetsLocalizations localizations) {
    return IOSSystemContextMenuItemDataSearchWeb(
      title: title ?? localizations.searchWebButtonLabel,
    );
  }

  @override
  String toString() {
    return 'IOSSystemContextMenuItemSearchWeb(title: $title)';
  }
}

/// Creates an instance of [IOSSystemContextMenuItem] for the
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
///  * [IOSSystemContextMenuItemDataShare], which specifies the data to be sent
///    to the platform for this same button.
final class IOSSystemContextMenuItemShare extends IOSSystemContextMenuItem {
  /// Creates an instance of [IOSSystemContextMenuItemShare].
  const IOSSystemContextMenuItemShare({this.title});

  @override
  final String? title;

  @override
  IOSSystemContextMenuItemDataShare getData(WidgetsLocalizations localizations) {
    return IOSSystemContextMenuItemDataShare(title: title ?? localizations.shareButtonLabel);
  }

  @override
  String toString() {
    return 'IOSSystemContextMenuItemShare(title: $title)';
  }
}

// TODO(justinmc): Support the "custom" type.
// https://github.com/flutter/flutter/issues/103163
