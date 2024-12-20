// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'app.dart';
/// @docImport 'material_localizations.dart';
/// @docImport 'selectable_text.dart';
library;

import 'package:flutter/cupertino.dart';
import 'package:flutter/rendering.dart';

import 'adaptive_text_selection_toolbar.dart';
import 'debug.dart';
import 'desktop_text_selection.dart';
import 'magnifier.dart';
import 'text_selection.dart';
import 'theme.dart';

/// A widget that introduces an area for user selections with adaptive selection
/// controls.
///
/// This widget creates a [SelectableRegion] with platform-adaptive selection
/// controls.
///
/// Flutter widgets are not selectable by default. To enable selection for
/// a specific screen, consider wrapping the body of the [Route] with a
/// [SelectionArea].
///
/// The [SelectionArea] widget must have a [Localizations] ancestor that
/// contains a [MaterialLocalizations] delegate; using the [MaterialApp] widget
/// ensures that such an ancestor is present.
///
/// {@tool dartpad}
/// This example shows how to make a screen selectable.
///
/// ** See code in examples/api/lib/material/selection_area/selection_area.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [SelectableRegion], which provides an overview of the selection system.
///  * [SelectableText], which enables selection on a single run of text.
///  * [SelectionListener], which enables accessing the [SelectionDetails] of
///    the selectable subtree it wraps.
class SelectionArea extends StatefulWidget {
  /// Creates a [SelectionArea].
  ///
  /// If [selectionControls] is null, a platform specific one is used.
  const SelectionArea({
    super.key,
    this.focusNode,
    this.selectionControls,
    this.contextMenuBuilder = _defaultContextMenuBuilder,
    this.magnifierConfiguration,
    this.onSelectionChanged,
    required this.child,
  });

  /// The configuration for the magnifier in the selection region.
  ///
  /// By default, builds a [CupertinoTextMagnifier] on iOS and [TextMagnifier]
  /// on Android, and builds nothing on all other platforms. To suppress the
  /// magnifier, consider passing [TextMagnifierConfiguration.disabled].
  ///
  /// {@macro flutter.widgets.magnifier.intro}
  final TextMagnifierConfiguration? magnifierConfiguration;

  /// {@macro flutter.widgets.Focus.focusNode}
  final FocusNode? focusNode;

  /// The delegate to build the selection handles and toolbar.
  ///
  /// If it is null, the platform specific selection control is used.
  final TextSelectionControls? selectionControls;

  /// {@macro flutter.widgets.EditableText.contextMenuBuilder}
  ///
  /// If not provided, will build a default menu based on the ambient
  /// [ThemeData.platform].
  ///
  /// {@tool dartpad}
  /// This example shows how to build a custom context menu for any selected
  /// content in a SelectionArea.
  ///
  /// ** See code in examples/api/lib/material/context_menu/selectable_region_toolbar_builder.0.dart **
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * [AdaptiveTextSelectionToolbar], which is built by default.
  final SelectableRegionContextMenuBuilder? contextMenuBuilder;

  /// Called when the selected content changes.
  final ValueChanged<SelectedContent?>? onSelectionChanged;

  /// The child widget this selection area applies to.
  ///
  /// {@macro flutter.widgets.ProxyWidget.child}
  final Widget child;

  static Widget _defaultContextMenuBuilder(
    BuildContext context,
    SelectableRegionState selectableRegionState,
  ) {
    return AdaptiveTextSelectionToolbar.selectableRegion(
      selectableRegionState: selectableRegionState,
    );
  }

  @override
  State<StatefulWidget> createState() => SelectionAreaState();
}

/// State for a [SelectionArea].
class SelectionAreaState extends State<SelectionArea> {
  final GlobalKey<SelectableRegionState> _selectableRegionKey = GlobalKey<SelectableRegionState>();

  /// The [State] of the [SelectableRegion] for which this [SelectionArea] wraps.
  SelectableRegionState get selectableRegion => _selectableRegionKey.currentState!;

  @protected
  @override
  Widget build(BuildContext context) {
    assert(debugCheckHasMaterialLocalizations(context));
    final TextSelectionControls controls =
        widget.selectionControls ??
        switch (Theme.of(context).platform) {
          TargetPlatform.android || TargetPlatform.fuchsia => materialTextSelectionHandleControls,
          TargetPlatform.linux || TargetPlatform.windows => desktopTextSelectionHandleControls,
          TargetPlatform.iOS => cupertinoTextSelectionHandleControls,
          TargetPlatform.macOS => cupertinoDesktopTextSelectionHandleControls,
        };
    return SelectableRegion(
      key: _selectableRegionKey,
      selectionControls: controls,
      focusNode: widget.focusNode,
      contextMenuBuilder: widget.contextMenuBuilder,
      magnifierConfiguration:
          widget.magnifierConfiguration ?? TextMagnifier.adaptiveMagnifierConfiguration,
      onSelectionChanged: widget.onSelectionChanged,
      child: widget.child,
    );
  }
}
