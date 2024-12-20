// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart' show ValueListenable, clampDouble;
import 'package:flutter/widgets.dart';

import 'desktop_text_selection_toolbar.dart';
import 'desktop_text_selection_toolbar_button.dart';
import 'localizations.dart';

/// MacOS Cupertino styled text selection handle controls.
///
/// Specifically does not manage the toolbar, which is left to
/// [EditableText.contextMenuBuilder].
class _CupertinoDesktopTextSelectionHandleControls extends CupertinoDesktopTextSelectionControls
    with TextSelectionHandleControls {}

/// Desktop Cupertino styled text selection controls.
///
/// The [cupertinoDesktopTextSelectionControls] global variable has a
/// suitable instance of this class.
class CupertinoDesktopTextSelectionControls extends TextSelectionControls {
  /// Desktop has no text selection handles.
  @override
  Size getHandleSize(double textLineHeight) {
    return Size.zero;
  }

  /// Builder for the MacOS-style copy/paste text selection toolbar.
  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  @override
  Widget buildToolbar(
    BuildContext context,
    Rect globalEditableRegion,
    double textLineHeight,
    Offset selectionMidpoint,
    List<TextSelectionPoint> endpoints,
    TextSelectionDelegate delegate,
    ValueListenable<ClipboardStatus>? clipboardStatus,
    Offset? lastSecondaryTapDownPosition,
  ) {
    return _CupertinoDesktopTextSelectionControlsToolbar(
      clipboardStatus: clipboardStatus,
      endpoints: endpoints,
      globalEditableRegion: globalEditableRegion,
      handleCut: canCut(delegate) ? () => handleCut(delegate) : null,
      handleCopy: canCopy(delegate) ? () => handleCopy(delegate) : null,
      handlePaste: canPaste(delegate) ? () => handlePaste(delegate) : null,
      handleSelectAll: canSelectAll(delegate) ? () => handleSelectAll(delegate) : null,
      selectionMidpoint: selectionMidpoint,
      lastSecondaryTapDownPosition: lastSecondaryTapDownPosition,
      textLineHeight: textLineHeight,
    );
  }

  /// Builds the text selection handles, but desktop has none.
  @override
  Widget buildHandle(
    BuildContext context,
    TextSelectionHandleType type,
    double textLineHeight, [
    VoidCallback? onTap,
  ]) {
    return const SizedBox.shrink();
  }

  /// Gets the position for the text selection handles, but desktop has none.
  @override
  Offset getHandleAnchor(TextSelectionHandleType type, double textLineHeight) {
    return Offset.zero;
  }

  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  @override
  void handleSelectAll(TextSelectionDelegate delegate) {
    super.handleSelectAll(delegate);
    delegate.hideToolbar();
  }
}

// TODO(justinmc): Deprecate this after TextSelectionControls.buildToolbar is
// deleted, when users should migrate back to
// cupertinoDesktopTextSelectionControls.
// See https://github.com/flutter/flutter/pull/124262
/// Text selection handle controls that follow MacOS design conventions.
final TextSelectionControls cupertinoDesktopTextSelectionHandleControls =
    _CupertinoDesktopTextSelectionHandleControls();

/// Text selection controls that follows MacOS design conventions.
final TextSelectionControls cupertinoDesktopTextSelectionControls =
    CupertinoDesktopTextSelectionControls();

// Generates the child that's passed into CupertinoDesktopTextSelectionToolbar.
class _CupertinoDesktopTextSelectionControlsToolbar extends StatefulWidget {
  const _CupertinoDesktopTextSelectionControlsToolbar({
    required this.clipboardStatus,
    required this.endpoints,
    required this.globalEditableRegion,
    required this.handleCopy,
    required this.handleCut,
    required this.handlePaste,
    required this.handleSelectAll,
    required this.selectionMidpoint,
    required this.textLineHeight,
    required this.lastSecondaryTapDownPosition,
  });

  final ValueListenable<ClipboardStatus>? clipboardStatus;
  final List<TextSelectionPoint> endpoints;
  final Rect globalEditableRegion;
  final VoidCallback? handleCopy;
  final VoidCallback? handleCut;
  final VoidCallback? handlePaste;
  final VoidCallback? handleSelectAll;
  final Offset? lastSecondaryTapDownPosition;
  final Offset selectionMidpoint;
  final double textLineHeight;

  @override
  _CupertinoDesktopTextSelectionControlsToolbarState createState() =>
      _CupertinoDesktopTextSelectionControlsToolbarState();
}

class _CupertinoDesktopTextSelectionControlsToolbarState
    extends State<_CupertinoDesktopTextSelectionControlsToolbar> {
  void _onChangedClipboardStatus() {
    setState(() {
      // Inform the widget that the value of clipboardStatus has changed.
    });
  }

  @override
  void initState() {
    super.initState();
    widget.clipboardStatus?.addListener(_onChangedClipboardStatus);
  }

  @override
  void didUpdateWidget(_CupertinoDesktopTextSelectionControlsToolbar oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.clipboardStatus != widget.clipboardStatus) {
      oldWidget.clipboardStatus?.removeListener(_onChangedClipboardStatus);
      widget.clipboardStatus?.addListener(_onChangedClipboardStatus);
    }
  }

  @override
  void dispose() {
    widget.clipboardStatus?.removeListener(_onChangedClipboardStatus);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    // Don't render the menu until the state of the clipboard is known.
    if (widget.handlePaste != null && widget.clipboardStatus?.value == ClipboardStatus.unknown) {
      return const SizedBox.shrink();
    }

    assert(debugCheckHasMediaQuery(context));
    final EdgeInsets mediaQueryPadding = MediaQuery.paddingOf(context);

    final Offset midpointAnchor = Offset(
      clampDouble(
        widget.selectionMidpoint.dx - widget.globalEditableRegion.left,
        mediaQueryPadding.left,
        MediaQuery.sizeOf(context).width - mediaQueryPadding.right,
      ),
      widget.selectionMidpoint.dy - widget.globalEditableRegion.top,
    );

    final List<Widget> items = <Widget>[];
    final CupertinoLocalizations localizations = CupertinoLocalizations.of(context);
    final Widget onePhysicalPixelVerticalDivider = SizedBox(
      width: 1.0 / MediaQuery.devicePixelRatioOf(context),
    );

    void addToolbarButton(String text, VoidCallback onPressed) {
      if (items.isNotEmpty) {
        items.add(onePhysicalPixelVerticalDivider);
      }

      items.add(CupertinoDesktopTextSelectionToolbarButton.text(onPressed: onPressed, text: text));
    }

    if (widget.handleCut != null) {
      addToolbarButton(localizations.cutButtonLabel, widget.handleCut!);
    }
    if (widget.handleCopy != null) {
      addToolbarButton(localizations.copyButtonLabel, widget.handleCopy!);
    }
    if (widget.handlePaste != null && widget.clipboardStatus?.value == ClipboardStatus.pasteable) {
      addToolbarButton(localizations.pasteButtonLabel, widget.handlePaste!);
    }
    if (widget.handleSelectAll != null) {
      addToolbarButton(localizations.selectAllButtonLabel, widget.handleSelectAll!);
    }

    // If there is no option available, build an empty widget.
    if (items.isEmpty) {
      return const SizedBox.shrink();
    }

    return CupertinoDesktopTextSelectionToolbar(
      anchor: widget.lastSecondaryTapDownPosition ?? midpointAnchor,
      children: items,
    );
  }
}
