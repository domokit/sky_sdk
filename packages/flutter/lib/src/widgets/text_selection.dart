// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/cupertino.dart';
/// @docImport 'package:flutter/material.dart';
library;

import 'dart:async';
import 'dart:math' as math;

import 'package:characters/characters.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';

import 'basic.dart';
import 'binding.dart';
import 'constants.dart';
import 'context_menu_controller.dart';
import 'debug.dart';
import 'editable_text.dart';
import 'feedback.dart';
import 'framework.dart';
import 'gesture_detector.dart';
import 'inherited_theme.dart';
import 'magnifier.dart';
import 'overlay.dart';
import 'scrollable.dart';
import 'tap_region.dart';
import 'ticker_provider.dart';
import 'transitions.dart';

export 'package:flutter/rendering.dart' show TextSelectionPoint;
export 'package:flutter/services.dart' show TextSelectionDelegate;

/// The type for a Function that builds a toolbar's container with the given
/// child.
///
/// See also:
///
///   * [TextSelectionToolbar.toolbarBuilder], which is of this type.
///     type.
///   * [CupertinoTextSelectionToolbar.toolbarBuilder], which is similar, but
///     for a Cupertino-style toolbar.
typedef ToolbarBuilder = Widget Function(BuildContext context, Widget child);

/// ParentData that determines whether or not to paint the corresponding child.
///
/// Used in the layout of the Cupertino and Material text selection menus, which
/// decide whether or not to paint their buttons after laying them out and
/// determining where they overflow.
class ToolbarItemsParentData extends ContainerBoxParentData<RenderBox> {
  /// Whether or not this child is painted.
  ///
  /// Children in the selection toolbar may be laid out for measurement purposes
  /// but not painted. This allows these children to be identified.
  bool shouldPaint = false;

  @override
  String toString() => '${super.toString()}; shouldPaint=$shouldPaint';
}

/// An interface for building the selection UI, to be provided by the
/// implementer of the toolbar widget.
///
/// Parts of this class, including [buildToolbar], have been deprecated in favor
/// of [EditableText.contextMenuBuilder], which is now the preferred way to
/// customize the context menus.
///
/// ## Use with [EditableText.contextMenuBuilder]
///
/// For backwards compatibility during the deprecation period, when
/// [EditableText.selectionControls] is set to an object that does not mix in
/// [TextSelectionHandleControls], [EditableText.contextMenuBuilder] is ignored
/// in favor of the deprecated [buildToolbar].
///
/// To migrate code from [buildToolbar] to the preferred
/// [EditableText.contextMenuBuilder], while still using [buildHandle], mix in
/// [TextSelectionHandleControls] into the [TextSelectionControls] subclass when
/// moving any toolbar code to a callback passed to
/// [EditableText.contextMenuBuilder].
///
/// In due course, [buildToolbar] will be removed, and the mixin will no longer
/// be necessary as a way to flag to the framework that the code has been
/// migrated and does not expect [buildToolbar] to be called.
///
/// For more information, see <https://docs.flutter.dev/release/breaking-changes/context-menus>.
///
/// See also:
///
///  * [SelectionArea], which selects appropriate text selection controls
///    based on the current platform.
abstract class TextSelectionControls {
  /// Builds a selection handle of the given `type`.
  ///
  /// The top left corner of this widget is positioned at the bottom of the
  /// selection position.
  ///
  /// The supplied [onTap] should be invoked when the handle is tapped, if such
  /// interaction is allowed. As a counterexample, the default selection handle
  /// on iOS [cupertinoTextSelectionControls] does not call [onTap] at all,
  /// since its handles are not meant to be tapped.
  Widget buildHandle(
    BuildContext context,
    TextSelectionHandleType type,
    double textLineHeight, [
    VoidCallback? onTap,
  ]);

  /// Get the anchor point of the handle relative to itself. The anchor point is
  /// the point that is aligned with a specific point in the text. A handle
  /// often visually "points to" that location.
  Offset getHandleAnchor(TextSelectionHandleType type, double textLineHeight);

  /// Builds a toolbar near a text selection.
  ///
  /// Typically displays buttons for copying and pasting text.
  ///
  /// The [globalEditableRegion] parameter is the TextField size of the global
  /// coordinate system in logical pixels.
  ///
  /// The [textLineHeight] parameter is the [RenderEditable.preferredLineHeight]
  /// of the [RenderEditable] we are building a toolbar for.
  ///
  /// The [selectionMidpoint] parameter is a general calculation midpoint
  /// parameter of the toolbar. More detailed position information
  /// is computable from the [endpoints] parameter.
  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  Widget buildToolbar(
    BuildContext context,
    Rect globalEditableRegion,
    double textLineHeight,
    Offset selectionMidpoint,
    List<TextSelectionPoint> endpoints,
    TextSelectionDelegate delegate,
    ValueListenable<ClipboardStatus>? clipboardStatus,
    Offset? lastSecondaryTapDownPosition,
  );

  /// Returns the size of the selection handle.
  Size getHandleSize(double textLineHeight);

  /// Whether the current selection of the text field managed by the given
  /// `delegate` can be removed from the text field and placed into the
  /// [Clipboard].
  ///
  /// By default, false is returned when nothing is selected in the text field.
  ///
  /// Subclasses can use this to decide if they should expose the cut
  /// functionality to the user.
  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  bool canCut(TextSelectionDelegate delegate) {
    return delegate.cutEnabled && !delegate.textEditingValue.selection.isCollapsed;
  }

  /// Whether the current selection of the text field managed by the given
  /// `delegate` can be copied to the [Clipboard].
  ///
  /// By default, false is returned when nothing is selected in the text field.
  ///
  /// Subclasses can use this to decide if they should expose the copy
  /// functionality to the user.
  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  bool canCopy(TextSelectionDelegate delegate) {
    return delegate.copyEnabled && !delegate.textEditingValue.selection.isCollapsed;
  }

  /// Whether the text field managed by the given `delegate` supports pasting
  /// from the clipboard.
  ///
  /// Subclasses can use this to decide if they should expose the paste
  /// functionality to the user.
  ///
  /// This does not consider the contents of the clipboard. Subclasses may want
  /// to, for example, disallow pasting when the clipboard contains an empty
  /// string.
  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  bool canPaste(TextSelectionDelegate delegate) {
    return delegate.pasteEnabled;
  }

  /// Whether the current selection of the text field managed by the given
  /// `delegate` can be extended to include the entire content of the text
  /// field.
  ///
  /// Subclasses can use this to decide if they should expose the select all
  /// functionality to the user.
  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  bool canSelectAll(TextSelectionDelegate delegate) {
    return delegate.selectAllEnabled &&
        delegate.textEditingValue.text.isNotEmpty &&
        delegate.textEditingValue.selection.isCollapsed;
  }

  /// Call [TextSelectionDelegate.cutSelection] to cut current selection.
  ///
  /// This is called by subclasses when their cut affordance is activated by
  /// the user.
  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  void handleCut(TextSelectionDelegate delegate) {
    delegate.cutSelection(SelectionChangedCause.toolbar);
  }

  /// Call [TextSelectionDelegate.copySelection] to copy current selection.
  ///
  /// This is called by subclasses when their copy affordance is activated by
  /// the user.
  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  void handleCopy(TextSelectionDelegate delegate) {
    delegate.copySelection(SelectionChangedCause.toolbar);
  }

  /// Call [TextSelectionDelegate.pasteText] to paste text.
  ///
  /// This is called by subclasses when their paste affordance is activated by
  /// the user.
  ///
  /// This function is asynchronous since interacting with the clipboard is
  /// asynchronous. Race conditions may exist with this API as currently
  /// implemented.
  // TODO(ianh): https://github.com/flutter/flutter/issues/11427
  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  Future<void> handlePaste(TextSelectionDelegate delegate) async {
    delegate.pasteText(SelectionChangedCause.toolbar);
  }

  /// Call [TextSelectionDelegate.selectAll] to set the current selection to
  /// contain the entire text value.
  ///
  /// Does not hide the toolbar.
  ///
  /// This is called by subclasses when their select-all affordance is activated
  /// by the user.
  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  void handleSelectAll(TextSelectionDelegate delegate) {
    delegate.selectAll(SelectionChangedCause.toolbar);
  }
}

/// Text selection controls that do not show any toolbars or handles.
///
/// This is a placeholder, suitable for temporary use during development, but
/// not practical for production. For example, it provides no way for the user
/// to interact with selections: no context menus on desktop, no toolbars or
/// drag handles on mobile, etc. For production, consider using
/// [MaterialTextSelectionControls] or creating a custom subclass of
/// [TextSelectionControls].
///
/// The [emptyTextSelectionControls] global variable has a
/// suitable instance of this class.
class EmptyTextSelectionControls extends TextSelectionControls {
  @override
  Size getHandleSize(double textLineHeight) => Size.zero;

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
  ) => const SizedBox.shrink();

  @override
  Widget buildHandle(
    BuildContext context,
    TextSelectionHandleType type,
    double textLineHeight, [
    VoidCallback? onTap,
  ]) {
    return const SizedBox.shrink();
  }

  @override
  Offset getHandleAnchor(TextSelectionHandleType type, double textLineHeight) {
    return Offset.zero;
  }
}

/// Text selection controls that do not show any toolbars or handles.
///
/// This is a placeholder, suitable for temporary use during development, but
/// not practical for production. For example, it provides no way for the user
/// to interact with selections: no context menus on desktop, no toolbars or
/// drag handles on mobile, etc. For production, consider using
/// [materialTextSelectionControls] or creating a custom subclass of
/// [TextSelectionControls].
final TextSelectionControls emptyTextSelectionControls = EmptyTextSelectionControls();

/// An object that manages a pair of text selection handles for a
/// [RenderEditable].
///
/// This class is a wrapper of [SelectionOverlay] to provide APIs specific for
/// [RenderEditable]s. To manage selection handles for custom widgets, use
/// [SelectionOverlay] instead.
class TextSelectionOverlay {
  /// Creates an object that manages overlay entries for selection handles.
  ///
  /// The [context] must have an [Overlay] as an ancestor.
  TextSelectionOverlay({
    required TextEditingValue value,
    required this.context,
    Widget? debugRequiredFor,
    required LayerLink toolbarLayerLink,
    required LayerLink startHandleLayerLink,
    required LayerLink endHandleLayerLink,
    required this.renderObject,
    this.selectionControls,
    bool handlesVisible = false,
    required this.selectionDelegate,
    DragStartBehavior dragStartBehavior = DragStartBehavior.start,
    VoidCallback? onSelectionHandleTapped,
    ClipboardStatusNotifier? clipboardStatus,
    this.contextMenuBuilder,
    required TextMagnifierConfiguration magnifierConfiguration,
  }) : _handlesVisible = handlesVisible,
       _value = value {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: 'package:flutter/widgets.dart',
        className: '$TextSelectionOverlay',
        object: this,
      );
    }
    renderObject.selectionStartInViewport.addListener(_updateTextSelectionOverlayVisibilities);
    renderObject.selectionEndInViewport.addListener(_updateTextSelectionOverlayVisibilities);
    _updateTextSelectionOverlayVisibilities();
    _selectionOverlay = SelectionOverlay(
      magnifierConfiguration: magnifierConfiguration,
      context: context,
      debugRequiredFor: debugRequiredFor,
      // The metrics will be set when show handles.
      startHandleType: TextSelectionHandleType.collapsed,
      startHandlesVisible: _effectiveStartHandleVisibility,
      lineHeightAtStart: 0.0,
      onStartHandleDragStart: _handleSelectionStartHandleDragStart,
      onStartHandleDragUpdate: _handleSelectionStartHandleDragUpdate,
      onEndHandleDragEnd: _handleAnyDragEnd,
      endHandleType: TextSelectionHandleType.collapsed,
      endHandlesVisible: _effectiveEndHandleVisibility,
      lineHeightAtEnd: 0.0,
      onEndHandleDragStart: _handleSelectionEndHandleDragStart,
      onEndHandleDragUpdate: _handleSelectionEndHandleDragUpdate,
      onStartHandleDragEnd: _handleAnyDragEnd,
      toolbarVisible: _effectiveToolbarVisibility,
      selectionEndpoints: const <TextSelectionPoint>[],
      selectionControls: selectionControls,
      selectionDelegate: selectionDelegate,
      clipboardStatus: clipboardStatus,
      startHandleLayerLink: startHandleLayerLink,
      endHandleLayerLink: endHandleLayerLink,
      toolbarLayerLink: toolbarLayerLink,
      onSelectionHandleTapped: onSelectionHandleTapped,
      dragStartBehavior: dragStartBehavior,
      toolbarLocation: renderObject.lastSecondaryTapDownPosition,
    );
  }

  /// {@template flutter.widgets.SelectionOverlay.context}
  /// The context in which the selection UI should appear.
  ///
  /// This context must have an [Overlay] as an ancestor because this object
  /// will display the text selection handles in that [Overlay].
  /// {@endtemplate}
  final BuildContext context;

  // TODO(mpcomplete): what if the renderObject is removed or replaced, or
  // moves? Not sure what cases I need to handle, or how to handle them.
  /// The editable line in which the selected text is being displayed.
  final RenderEditable renderObject;

  /// {@macro flutter.widgets.SelectionOverlay.selectionControls}
  final TextSelectionControls? selectionControls;

  /// {@macro flutter.widgets.SelectionOverlay.selectionDelegate}
  final TextSelectionDelegate selectionDelegate;

  late final SelectionOverlay _selectionOverlay;

  /// {@macro flutter.widgets.EditableText.contextMenuBuilder}
  ///
  /// If not provided, no context menu will be built.
  final WidgetBuilder? contextMenuBuilder;

  /// Retrieve current value.
  @visibleForTesting
  TextEditingValue get value => _value;

  TextEditingValue _value;

  TextSelection get _selection => _value.selection;

  final ValueNotifier<bool> _effectiveStartHandleVisibility = ValueNotifier<bool>(false);
  final ValueNotifier<bool> _effectiveEndHandleVisibility = ValueNotifier<bool>(false);
  final ValueNotifier<bool> _effectiveToolbarVisibility = ValueNotifier<bool>(false);

  void _updateTextSelectionOverlayVisibilities() {
    _effectiveStartHandleVisibility.value =
        _handlesVisible && renderObject.selectionStartInViewport.value;
    _effectiveEndHandleVisibility.value =
        _handlesVisible && renderObject.selectionEndInViewport.value;
    _effectiveToolbarVisibility.value =
        renderObject.selectionStartInViewport.value || renderObject.selectionEndInViewport.value;
  }

  /// Whether selection handles are visible.
  ///
  /// Set to false if you want to hide the handles. Use this property to show or
  /// hide the handle without rebuilding them.
  ///
  /// Defaults to false.
  bool get handlesVisible => _handlesVisible;
  bool _handlesVisible = false;
  set handlesVisible(bool visible) {
    if (_handlesVisible == visible) {
      return;
    }
    _handlesVisible = visible;
    _updateTextSelectionOverlayVisibilities();
  }

  /// {@macro flutter.widgets.SelectionOverlay.showHandles}
  void showHandles() {
    _updateSelectionOverlay();
    _selectionOverlay.showHandles();
  }

  /// {@macro flutter.widgets.SelectionOverlay.hideHandles}
  void hideHandles() => _selectionOverlay.hideHandles();

  /// {@macro flutter.widgets.SelectionOverlay.showToolbar}
  void showToolbar() {
    _updateSelectionOverlay();

    if (selectionControls != null && selectionControls is! TextSelectionHandleControls) {
      _selectionOverlay.showToolbar();
      return;
    }

    if (contextMenuBuilder == null) {
      return;
    }

    assert(context.mounted);
    _selectionOverlay.showToolbar(context: context, contextMenuBuilder: contextMenuBuilder);
    return;
  }

  /// Shows toolbar with spell check suggestions of misspelled words that are
  /// available for click-and-replace.
  void showSpellCheckSuggestionsToolbar(WidgetBuilder spellCheckSuggestionsToolbarBuilder) {
    _updateSelectionOverlay();
    assert(context.mounted);
    _selectionOverlay.showSpellCheckSuggestionsToolbar(
      context: context,
      builder: spellCheckSuggestionsToolbarBuilder,
    );
    hideHandles();
  }

  /// {@macro flutter.widgets.SelectionOverlay.showMagnifier}
  void showMagnifier(Offset positionToShow) {
    final TextPosition position = renderObject.getPositionForPoint(positionToShow);
    _updateSelectionOverlay();
    _selectionOverlay.showMagnifier(
      _buildMagnifier(
        currentTextPosition: position,
        globalGesturePosition: positionToShow,
        renderEditable: renderObject,
      ),
    );
  }

  /// {@macro flutter.widgets.SelectionOverlay.updateMagnifier}
  void updateMagnifier(Offset positionToShow) {
    final TextPosition position = renderObject.getPositionForPoint(positionToShow);
    _updateSelectionOverlay();
    _selectionOverlay.updateMagnifier(
      _buildMagnifier(
        currentTextPosition: position,
        globalGesturePosition: positionToShow,
        renderEditable: renderObject,
      ),
    );
  }

  /// {@macro flutter.widgets.SelectionOverlay.hideMagnifier}
  void hideMagnifier() {
    _selectionOverlay.hideMagnifier();
  }

  /// Updates the overlay after the selection has changed.
  ///
  /// If this method is called while the [SchedulerBinding.schedulerPhase] is
  /// [SchedulerPhase.persistentCallbacks], i.e. during the build, layout, or
  /// paint phases (see [WidgetsBinding.drawFrame]), then the update is delayed
  /// until the post-frame callbacks phase. Otherwise the update is done
  /// synchronously. This means that it is safe to call during builds, but also
  /// that if you do call this during a build, the UI will not update until the
  /// next frame (i.e. many milliseconds later).
  void update(TextEditingValue newValue) {
    if (_value == newValue) {
      return;
    }
    _value = newValue;
    _updateSelectionOverlay();
    // _updateSelectionOverlay may not rebuild the selection overlay if the
    // text metrics and selection doesn't change even if the text has changed.
    // This rebuild is needed for the toolbar to update based on the latest text
    // value.
    _selectionOverlay.markNeedsBuild();
  }

  void _updateSelectionOverlay() {
    _selectionOverlay
      // Update selection handle metrics.
      ..startHandleType = _chooseType(
        renderObject.textDirection,
        TextSelectionHandleType.left,
        TextSelectionHandleType.right,
      )
      ..lineHeightAtStart = _getStartGlyphHeight()
      ..endHandleType = _chooseType(
        renderObject.textDirection,
        TextSelectionHandleType.right,
        TextSelectionHandleType.left,
      )
      ..lineHeightAtEnd = _getEndGlyphHeight()
      // Update selection toolbar metrics.
      ..selectionEndpoints = renderObject.getEndpointsForSelection(_selection)
      ..toolbarLocation = renderObject.lastSecondaryTapDownPosition;
  }

  /// Causes the overlay to update its rendering.
  ///
  /// This is intended to be called when the [renderObject] may have changed its
  /// text metrics (e.g. because the text was scrolled).
  void updateForScroll() {
    _updateSelectionOverlay();
    // This method may be called due to windows metrics changes. In that case,
    // non of the properties in _selectionOverlay will change, but a rebuild is
    // still needed.
    _selectionOverlay.markNeedsBuild();
  }

  /// Whether the handles are currently visible.
  bool get handlesAreVisible => _selectionOverlay._handles != null && handlesVisible;

  /// {@macro flutter.widgets.SelectionOverlay.toolbarIsVisible}
  ///
  /// See also:
  ///
  ///   * [spellCheckToolbarIsVisible], which is only whether the spell check menu
  ///     specifically is visible.
  bool get toolbarIsVisible => _selectionOverlay.toolbarIsVisible;

  /// Whether the magnifier is currently visible.
  bool get magnifierIsVisible => _selectionOverlay._magnifierController.shown;

  /// Whether the spell check menu is currently visible.
  ///
  /// See also:
  ///
  ///   * [toolbarIsVisible], which is whether any toolbar is visible.
  bool get spellCheckToolbarIsVisible => _selectionOverlay._spellCheckToolbarController.isShown;

  /// {@macro flutter.widgets.SelectionOverlay.hide}
  void hide() => _selectionOverlay.hide();

  /// {@macro flutter.widgets.SelectionOverlay.hideToolbar}
  void hideToolbar() => _selectionOverlay.hideToolbar();

  /// {@macro flutter.widgets.SelectionOverlay.dispose}
  void dispose() {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }
    _selectionOverlay.dispose();
    renderObject.selectionStartInViewport.removeListener(_updateTextSelectionOverlayVisibilities);
    renderObject.selectionEndInViewport.removeListener(_updateTextSelectionOverlayVisibilities);
    _effectiveToolbarVisibility.dispose();
    _effectiveStartHandleVisibility.dispose();
    _effectiveEndHandleVisibility.dispose();
    hideToolbar();
  }

  double _getStartGlyphHeight() {
    final String currText = selectionDelegate.textEditingValue.text;
    final int firstSelectedGraphemeExtent;
    Rect? startHandleRect;
    // Only calculate handle rects if the text in the previous frame
    // is the same as the text in the current frame. This is done because
    // widget.renderObject contains the renderEditable from the previous frame.
    // If the text changed between the current and previous frames then
    // widget.renderObject.getRectForComposingRange might fail. In cases where
    // the current frame is different from the previous we fall back to
    // renderObject.preferredLineHeight.
    if (renderObject.plainText == currText && _selection.isValid && !_selection.isCollapsed) {
      final String selectedGraphemes = _selection.textInside(currText);
      firstSelectedGraphemeExtent = selectedGraphemes.characters.first.length;
      startHandleRect = renderObject.getRectForComposingRange(
        TextRange(start: _selection.start, end: _selection.start + firstSelectedGraphemeExtent),
      );
    }
    return startHandleRect?.height ?? renderObject.preferredLineHeight;
  }

  double _getEndGlyphHeight() {
    final String currText = selectionDelegate.textEditingValue.text;
    final int lastSelectedGraphemeExtent;
    Rect? endHandleRect;
    // See the explanation in _getStartGlyphHeight.
    if (renderObject.plainText == currText && _selection.isValid && !_selection.isCollapsed) {
      final String selectedGraphemes = _selection.textInside(currText);
      lastSelectedGraphemeExtent = selectedGraphemes.characters.last.length;
      endHandleRect = renderObject.getRectForComposingRange(
        TextRange(start: _selection.end - lastSelectedGraphemeExtent, end: _selection.end),
      );
    }
    return endHandleRect?.height ?? renderObject.preferredLineHeight;
  }

  MagnifierInfo _buildMagnifier({
    required RenderEditable renderEditable,
    required Offset globalGesturePosition,
    required TextPosition currentTextPosition,
  }) {
    final TextSelection lineAtOffset = renderEditable.getLineAtOffset(currentTextPosition);
    final TextPosition positionAtEndOfLine = TextPosition(
      offset: lineAtOffset.extentOffset,
      affinity: TextAffinity.upstream,
    );

    // Default affinity is downstream.
    final TextPosition positionAtBeginningOfLine = TextPosition(offset: lineAtOffset.baseOffset);

    final Rect localLineBoundaries = Rect.fromPoints(
      renderEditable.getLocalRectForCaret(positionAtBeginningOfLine).topCenter,
      renderEditable.getLocalRectForCaret(positionAtEndOfLine).bottomCenter,
    );
    final RenderBox? overlay =
        Overlay.of(context, rootOverlay: true).context.findRenderObject() as RenderBox?;
    final Matrix4 transformToOverlay = renderEditable.getTransformTo(overlay);
    final Rect overlayLineBoundaries = MatrixUtils.transformRect(
      transformToOverlay,
      localLineBoundaries,
    );

    final Rect localCaretRect = renderEditable.getLocalRectForCaret(currentTextPosition);
    final Rect overlayCaretRect = MatrixUtils.transformRect(transformToOverlay, localCaretRect);

    final Offset overlayGesturePosition =
        overlay?.globalToLocal(globalGesturePosition) ?? globalGesturePosition;

    return MagnifierInfo(
      fieldBounds: MatrixUtils.transformRect(transformToOverlay, renderEditable.paintBounds),
      globalGesturePosition: overlayGesturePosition,
      caretRect: overlayCaretRect,
      currentLineBoundaries: overlayLineBoundaries,
    );
  }

  // The contact position of the gesture at the current end handle location, in
  // global coordinates. Updated when the handle moves.
  late double _endHandleDragPosition;

  // The distance from _endHandleDragPosition to the center of the line that it
  // corresponds to, in global coordinates.
  late double _endHandleDragTarget;

  // The initial selection when a selection handle drag has started.
  TextSelection? _dragStartSelection;

  void _handleSelectionEndHandleDragStart(DragStartDetails details) {
    if (!renderObject.attached) {
      return;
    }

    _endHandleDragPosition = details.globalPosition.dy;

    // Use local coordinates when dealing with line height. because in case of a
    // scale transformation, the line height will also be scaled.
    final double centerOfLineLocal =
        _selectionOverlay.selectionEndpoints.last.point.dy - renderObject.preferredLineHeight / 2;
    final double centerOfLineGlobal = renderObject.localToGlobal(Offset(0.0, centerOfLineLocal)).dy;
    _endHandleDragTarget = centerOfLineGlobal - details.globalPosition.dy;
    // Instead of finding the TextPosition at the handle's location directly,
    // use the vertical center of the line that it points to. This is because
    // selection handles typically hang above or below the line that they point
    // to.
    final TextPosition position = renderObject.getPositionForPoint(
      Offset(details.globalPosition.dx, centerOfLineGlobal),
    );
    _dragStartSelection ??= _selection;

    _selectionOverlay.showMagnifier(
      _buildMagnifier(
        currentTextPosition: position,
        globalGesturePosition: details.globalPosition,
        renderEditable: renderObject,
      ),
    );
  }

  /// Given a handle position and drag position, returns the position of handle
  /// after the drag.
  ///
  /// The handle jumps instantly between lines when the drag reaches a full
  /// line's height away from the original handle position. In other words, the
  /// line jump happens when the contact point would be located at the same
  /// place on the handle at the new line as when the gesture started, for both
  /// directions.
  ///
  /// This is not the same as just maintaining an offset from the target and the
  /// contact point. There is no point at which moving the drag up and down a
  /// small sub-line-height distance will cause the cursor to jump up and down
  /// between lines. The drag distance must be a full line height for the cursor
  /// to change lines, for both directions.
  ///
  /// Both parameters must be in local coordinates because the untransformed
  /// line height is used, and the return value is in local coordinates as well.
  double _getHandleDy(double dragDy, double handleDy) {
    final double distanceDragged = dragDy - handleDy;
    final int dragDirection = distanceDragged < 0.0 ? -1 : 1;
    final int linesDragged =
        dragDirection * (distanceDragged.abs() / renderObject.preferredLineHeight).floor();
    return handleDy + linesDragged * renderObject.preferredLineHeight;
  }

  void _handleSelectionEndHandleDragUpdate(DragUpdateDetails details) {
    if (!renderObject.attached) {
      return;
    }
    assert(_dragStartSelection != null);

    // This is NOT the same as details.localPosition. That is relative to the
    // selection handle, whereas this is relative to the RenderEditable.
    final Offset localPosition = renderObject.globalToLocal(details.globalPosition);

    final double nextEndHandleDragPositionLocal = _getHandleDy(
      localPosition.dy,
      renderObject.globalToLocal(Offset(0.0, _endHandleDragPosition)).dy,
    );
    _endHandleDragPosition =
        renderObject.localToGlobal(Offset(0.0, nextEndHandleDragPositionLocal)).dy;

    final Offset handleTargetGlobal = Offset(
      details.globalPosition.dx,
      _endHandleDragPosition + _endHandleDragTarget,
    );

    final TextPosition position = renderObject.getPositionForPoint(handleTargetGlobal);

    if (_dragStartSelection!.isCollapsed) {
      _selectionOverlay.updateMagnifier(
        _buildMagnifier(
          currentTextPosition: position,
          globalGesturePosition: details.globalPosition,
          renderEditable: renderObject,
        ),
      );

      final TextSelection currentSelection = TextSelection.fromPosition(position);
      _handleSelectionHandleChanged(currentSelection);
      return;
    }

    final TextSelection newSelection;
    switch (defaultTargetPlatform) {
      // On Apple platforms, dragging the base handle makes it the extent.
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        // Use this instead of _dragStartSelection.isNormalized because TextRange.isNormalized
        // always returns true for a TextSelection.
        final bool dragStartSelectionNormalized =
            _dragStartSelection!.extentOffset >= _dragStartSelection!.baseOffset;
        newSelection = TextSelection(
          baseOffset:
              dragStartSelectionNormalized
                  ? _dragStartSelection!.baseOffset
                  : _dragStartSelection!.extentOffset,
          extentOffset: position.offset,
        );
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        newSelection = TextSelection(
          baseOffset: _selection.baseOffset,
          extentOffset: position.offset,
        );
        if (newSelection.baseOffset >= newSelection.extentOffset) {
          return; // Don't allow order swapping.
        }
    }

    _handleSelectionHandleChanged(newSelection);

    _selectionOverlay.updateMagnifier(
      _buildMagnifier(
        currentTextPosition: newSelection.extent,
        globalGesturePosition: details.globalPosition,
        renderEditable: renderObject,
      ),
    );
  }

  // The contact position of the gesture at the current start handle location,
  // in global coordinates. Updated when the handle moves.
  late double _startHandleDragPosition;

  // The distance from _startHandleDragPosition to the center of the line that
  // it corresponds to, in global coordinates.
  late double _startHandleDragTarget;

  void _handleSelectionStartHandleDragStart(DragStartDetails details) {
    if (!renderObject.attached) {
      return;
    }

    _startHandleDragPosition = details.globalPosition.dy;

    // Use local coordinates when dealing with line height. because in case of a
    // scale transformation, the line height will also be scaled.
    final double centerOfLineLocal =
        _selectionOverlay.selectionEndpoints.first.point.dy - renderObject.preferredLineHeight / 2;
    final double centerOfLineGlobal = renderObject.localToGlobal(Offset(0.0, centerOfLineLocal)).dy;
    _startHandleDragTarget = centerOfLineGlobal - details.globalPosition.dy;
    // Instead of finding the TextPosition at the handle's location directly,
    // use the vertical center of the line that it points to. This is because
    // selection handles typically hang above or below the line that they point
    // to.
    final TextPosition position = renderObject.getPositionForPoint(
      Offset(details.globalPosition.dx, centerOfLineGlobal),
    );
    _dragStartSelection ??= _selection;

    _selectionOverlay.showMagnifier(
      _buildMagnifier(
        currentTextPosition: position,
        globalGesturePosition: details.globalPosition,
        renderEditable: renderObject,
      ),
    );
  }

  void _handleSelectionStartHandleDragUpdate(DragUpdateDetails details) {
    if (!renderObject.attached) {
      return;
    }
    assert(_dragStartSelection != null);

    // This is NOT the same as details.localPosition. That is relative to the
    // selection handle, whereas this is relative to the RenderEditable.
    final Offset localPosition = renderObject.globalToLocal(details.globalPosition);
    final double nextStartHandleDragPositionLocal = _getHandleDy(
      localPosition.dy,
      renderObject.globalToLocal(Offset(0.0, _startHandleDragPosition)).dy,
    );
    _startHandleDragPosition =
        renderObject.localToGlobal(Offset(0.0, nextStartHandleDragPositionLocal)).dy;
    final Offset handleTargetGlobal = Offset(
      details.globalPosition.dx,
      _startHandleDragPosition + _startHandleDragTarget,
    );
    final TextPosition position = renderObject.getPositionForPoint(handleTargetGlobal);

    if (_dragStartSelection!.isCollapsed) {
      _selectionOverlay.updateMagnifier(
        _buildMagnifier(
          currentTextPosition: position,
          globalGesturePosition: details.globalPosition,
          renderEditable: renderObject,
        ),
      );

      final TextSelection currentSelection = TextSelection.fromPosition(position);
      _handleSelectionHandleChanged(currentSelection);
      return;
    }

    final TextSelection newSelection;
    switch (defaultTargetPlatform) {
      // On Apple platforms, dragging the base handle makes it the extent.
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        // Use this instead of _dragStartSelection.isNormalized because TextRange.isNormalized
        // always returns true for a TextSelection.
        final bool dragStartSelectionNormalized =
            _dragStartSelection!.extentOffset >= _dragStartSelection!.baseOffset;
        newSelection = TextSelection(
          baseOffset:
              dragStartSelectionNormalized
                  ? _dragStartSelection!.extentOffset
                  : _dragStartSelection!.baseOffset,
          extentOffset: position.offset,
        );
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        newSelection = TextSelection(
          baseOffset: position.offset,
          extentOffset: _selection.extentOffset,
        );
        if (newSelection.baseOffset >= newSelection.extentOffset) {
          return; // Don't allow order swapping.
        }
    }

    _selectionOverlay.updateMagnifier(
      _buildMagnifier(
        currentTextPosition:
            newSelection.extent.offset < newSelection.base.offset
                ? newSelection.extent
                : newSelection.base,
        globalGesturePosition: details.globalPosition,
        renderEditable: renderObject,
      ),
    );

    _handleSelectionHandleChanged(newSelection);
  }

  void _handleAnyDragEnd(DragEndDetails details) {
    if (!context.mounted) {
      return;
    }
    _dragStartSelection = null;
    if (selectionControls is! TextSelectionHandleControls) {
      _selectionOverlay.hideMagnifier();
      if (!_selection.isCollapsed) {
        _selectionOverlay.showToolbar();
      }
      return;
    }
    _selectionOverlay.hideMagnifier();
    if (!_selection.isCollapsed) {
      _selectionOverlay.showToolbar(context: context, contextMenuBuilder: contextMenuBuilder);
    }
  }

  void _handleSelectionHandleChanged(TextSelection newSelection) {
    selectionDelegate.userUpdateTextEditingValue(
      _value.copyWith(selection: newSelection),
      SelectionChangedCause.drag,
    );
  }

  TextSelectionHandleType _chooseType(
    TextDirection textDirection,
    TextSelectionHandleType ltrType,
    TextSelectionHandleType rtlType,
  ) {
    if (_selection.isCollapsed) {
      return TextSelectionHandleType.collapsed;
    }

    return switch (textDirection) {
      TextDirection.ltr => ltrType,
      TextDirection.rtl => rtlType,
    };
  }
}

/// An object that manages a pair of selection handles and a toolbar.
///
/// The selection handles are displayed in the [Overlay] that most closely
/// encloses the given [BuildContext].
class SelectionOverlay {
  /// Creates an object that manages overlay entries for selection handles.
  ///
  /// The [context] must have an [Overlay] as an ancestor.
  SelectionOverlay({
    required this.context,
    this.debugRequiredFor,
    required TextSelectionHandleType startHandleType,
    required double lineHeightAtStart,
    this.startHandlesVisible,
    this.onStartHandleDragStart,
    this.onStartHandleDragUpdate,
    this.onStartHandleDragEnd,
    required TextSelectionHandleType endHandleType,
    required double lineHeightAtEnd,
    this.endHandlesVisible,
    this.onEndHandleDragStart,
    this.onEndHandleDragUpdate,
    this.onEndHandleDragEnd,
    this.toolbarVisible,
    required List<TextSelectionPoint> selectionEndpoints,
    required this.selectionControls,
    @Deprecated(
      'Use `contextMenuBuilder` in `showToolbar` instead. '
      'This feature was deprecated after v3.3.0-0.5.pre.',
    )
    required this.selectionDelegate,
    required this.clipboardStatus,
    required this.startHandleLayerLink,
    required this.endHandleLayerLink,
    required this.toolbarLayerLink,
    this.dragStartBehavior = DragStartBehavior.start,
    this.onSelectionHandleTapped,
    @Deprecated(
      'Use `contextMenuBuilder` in `showToolbar` instead. '
      'This feature was deprecated after v3.3.0-0.5.pre.',
    )
    Offset? toolbarLocation,
    this.magnifierConfiguration = TextMagnifierConfiguration.disabled,
  }) : _startHandleType = startHandleType,
       _lineHeightAtStart = lineHeightAtStart,
       _endHandleType = endHandleType,
       _lineHeightAtEnd = lineHeightAtEnd,
       _selectionEndpoints = selectionEndpoints,
       _toolbarLocation = toolbarLocation,
       assert(debugCheckHasOverlay(context)) {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: 'package:flutter/widgets.dart',
        className: '$SelectionOverlay',
        object: this,
      );
    }
  }

  /// {@macro flutter.widgets.SelectionOverlay.context}
  final BuildContext context;

  final ValueNotifier<MagnifierInfo> _magnifierInfo = ValueNotifier<MagnifierInfo>(
    MagnifierInfo.empty,
  );

  // [MagnifierController.show] and [MagnifierController.hide] should not be
  // called directly, except from inside [showMagnifier] and [hideMagnifier]. If
  // it is desired to show or hide the magnifier, call [showMagnifier] or
  // [hideMagnifier]. This is because the magnifier needs to orchestrate with
  // other properties in [SelectionOverlay].
  final MagnifierController _magnifierController = MagnifierController();

  /// The configuration for the magnifier.
  ///
  /// By default, [SelectionOverlay]'s [TextMagnifierConfiguration] is disabled.
  ///
  /// {@macro flutter.widgets.magnifier.intro}
  final TextMagnifierConfiguration magnifierConfiguration;

  /// {@template flutter.widgets.SelectionOverlay.toolbarIsVisible}
  /// Whether the toolbar is currently visible.
  ///
  /// Includes both the text selection toolbar and the spell check menu.
  /// {@endtemplate}
  bool get toolbarIsVisible {
    return selectionControls is TextSelectionHandleControls
        ? _contextMenuController.isShown || _spellCheckToolbarController.isShown
        : _toolbar != null || _spellCheckToolbarController.isShown;
  }

  /// {@template flutter.widgets.SelectionOverlay.showMagnifier}
  /// Shows the magnifier, and hides the toolbar if it was showing when [showMagnifier]
  /// was called. This is safe to call on platforms not mobile, since
  /// a magnifierBuilder will not be provided, or the magnifierBuilder will return null
  /// on platforms not mobile.
  ///
  /// This is NOT the source of truth for if the magnifier is up or not,
  /// since magnifiers may hide themselves. If this info is needed, check
  /// [MagnifierController.shown].
  /// {@endtemplate}
  void showMagnifier(MagnifierInfo initialMagnifierInfo) {
    if (toolbarIsVisible) {
      hideToolbar();
    }

    // Start from empty, so we don't utilize any remnant values.
    _magnifierInfo.value = initialMagnifierInfo;

    // Pre-build the magnifiers so we can tell if we've built something
    // or not. If we don't build a magnifiers, then we should not
    // insert anything in the overlay.
    final Widget? builtMagnifier = magnifierConfiguration.magnifierBuilder(
      context,
      _magnifierController,
      _magnifierInfo,
    );

    if (builtMagnifier == null) {
      return;
    }

    _magnifierController.show(
      context: context,
      below: magnifierConfiguration.shouldDisplayHandlesInMagnifier ? null : _handles?.start,
      builder: (_) => builtMagnifier,
    );
  }

  /// {@template flutter.widgets.SelectionOverlay.hideMagnifier}
  /// Hide the current magnifier.
  ///
  /// This does nothing if there is no magnifier.
  /// {@endtemplate}
  void hideMagnifier() {
    // This cannot be a check on `MagnifierController.shown`, since
    // it's possible that the magnifier is still in the overlay, but
    // not shown in cases where the magnifier hides itself.
    if (_magnifierController.overlayEntry == null) {
      return;
    }

    _magnifierController.hide();
  }

  /// The type of start selection handle.
  ///
  /// Changing the value while the handles are visible causes them to rebuild.
  TextSelectionHandleType get startHandleType => _startHandleType;
  TextSelectionHandleType _startHandleType;
  set startHandleType(TextSelectionHandleType value) {
    if (_startHandleType == value) {
      return;
    }
    _startHandleType = value;
    markNeedsBuild();
  }

  /// The line height at the selection start.
  ///
  /// This value is used for calculating the size of the start selection handle.
  ///
  /// Changing the value while the handles are visible causes them to rebuild.
  double get lineHeightAtStart => _lineHeightAtStart;
  double _lineHeightAtStart;
  set lineHeightAtStart(double value) {
    if (_lineHeightAtStart == value) {
      return;
    }
    _lineHeightAtStart = value;
    markNeedsBuild();
  }

  bool _isDraggingStartHandle = false;

  /// Whether the start handle is visible.
  ///
  /// If the value changes, the start handle uses [FadeTransition] to transition
  /// itself on and off the screen.
  ///
  /// If this is null, the start selection handle will always be visible.
  final ValueListenable<bool>? startHandlesVisible;

  /// Called when the users start dragging the start selection handles.
  final ValueChanged<DragStartDetails>? onStartHandleDragStart;

  void _handleStartHandleDragStart(DragStartDetails details) {
    assert(!_isDraggingStartHandle);
    // Calling OverlayEntry.remove may not happen until the following frame, so
    // it's possible for the handles to receive a gesture after calling remove.
    if (_handles == null) {
      _isDraggingStartHandle = false;
      return;
    }
    _isDraggingStartHandle = details.kind == PointerDeviceKind.touch;
    onStartHandleDragStart?.call(details);
  }

  void _handleStartHandleDragUpdate(DragUpdateDetails details) {
    // Calling OverlayEntry.remove may not happen until the following frame, so
    // it's possible for the handles to receive a gesture after calling remove.
    if (_handles == null) {
      _isDraggingStartHandle = false;
      return;
    }
    onStartHandleDragUpdate?.call(details);
  }

  /// Called when the users drag the start selection handles to new locations.
  final ValueChanged<DragUpdateDetails>? onStartHandleDragUpdate;

  /// Called when the users lift their fingers after dragging the start selection
  /// handles.
  final ValueChanged<DragEndDetails>? onStartHandleDragEnd;

  void _handleStartHandleDragEnd(DragEndDetails details) {
    _isDraggingStartHandle = false;
    // Calling OverlayEntry.remove may not happen until the following frame, so
    // it's possible for the handles to receive a gesture after calling remove.
    if (_handles == null) {
      return;
    }
    onStartHandleDragEnd?.call(details);
  }

  /// The type of end selection handle.
  ///
  /// Changing the value while the handles are visible causes them to rebuild.
  TextSelectionHandleType get endHandleType => _endHandleType;
  TextSelectionHandleType _endHandleType;
  set endHandleType(TextSelectionHandleType value) {
    if (_endHandleType == value) {
      return;
    }
    _endHandleType = value;
    markNeedsBuild();
  }

  /// The line height at the selection end.
  ///
  /// This value is used for calculating the size of the end selection handle.
  ///
  /// Changing the value while the handles are visible causes them to rebuild.
  double get lineHeightAtEnd => _lineHeightAtEnd;
  double _lineHeightAtEnd;
  set lineHeightAtEnd(double value) {
    if (_lineHeightAtEnd == value) {
      return;
    }
    _lineHeightAtEnd = value;
    markNeedsBuild();
  }

  bool _isDraggingEndHandle = false;

  /// Whether the end handle is visible.
  ///
  /// If the value changes, the end handle uses [FadeTransition] to transition
  /// itself on and off the screen.
  ///
  /// If this is null, the end selection handle will always be visible.
  final ValueListenable<bool>? endHandlesVisible;

  /// Called when the users start dragging the end selection handles.
  final ValueChanged<DragStartDetails>? onEndHandleDragStart;

  void _handleEndHandleDragStart(DragStartDetails details) {
    assert(!_isDraggingEndHandle);
    // Calling OverlayEntry.remove may not happen until the following frame, so
    // it's possible for the handles to receive a gesture after calling remove.
    if (_handles == null) {
      _isDraggingEndHandle = false;
      return;
    }
    _isDraggingEndHandle = details.kind == PointerDeviceKind.touch;
    onEndHandleDragStart?.call(details);
  }

  void _handleEndHandleDragUpdate(DragUpdateDetails details) {
    // Calling OverlayEntry.remove may not happen until the following frame, so
    // it's possible for the handles to receive a gesture after calling remove.
    if (_handles == null) {
      _isDraggingEndHandle = false;
      return;
    }
    onEndHandleDragUpdate?.call(details);
  }

  /// Called when the users drag the end selection handles to new locations.
  final ValueChanged<DragUpdateDetails>? onEndHandleDragUpdate;

  /// Called when the users lift their fingers after dragging the end selection
  /// handles.
  final ValueChanged<DragEndDetails>? onEndHandleDragEnd;

  void _handleEndHandleDragEnd(DragEndDetails details) {
    _isDraggingEndHandle = false;
    // Calling OverlayEntry.remove may not happen until the following frame, so
    // it's possible for the handles to receive a gesture after calling remove.
    if (_handles == null) {
      return;
    }
    onEndHandleDragEnd?.call(details);
  }

  /// Whether the toolbar is visible.
  ///
  /// If the value changes, the toolbar uses [FadeTransition] to transition
  /// itself on and off the screen.
  ///
  /// If this is null the toolbar will always be visible.
  final ValueListenable<bool>? toolbarVisible;

  /// The text selection positions of selection start and end.
  List<TextSelectionPoint> get selectionEndpoints => _selectionEndpoints;
  List<TextSelectionPoint> _selectionEndpoints;
  set selectionEndpoints(List<TextSelectionPoint> value) {
    if (!listEquals(_selectionEndpoints, value)) {
      markNeedsBuild();
      if (_isDraggingEndHandle || _isDraggingStartHandle) {
        switch (defaultTargetPlatform) {
          case TargetPlatform.android:
            HapticFeedback.selectionClick();
          case TargetPlatform.fuchsia:
          case TargetPlatform.iOS:
          case TargetPlatform.linux:
          case TargetPlatform.macOS:
          case TargetPlatform.windows:
            break;
        }
      }
    }
    _selectionEndpoints = value;
  }

  /// Debugging information for explaining why the [Overlay] is required.
  final Widget? debugRequiredFor;

  /// The object supplied to the [CompositedTransformTarget] that wraps the text
  /// field.
  final LayerLink toolbarLayerLink;

  /// The objects supplied to the [CompositedTransformTarget] that wraps the
  /// location of start selection handle.
  final LayerLink startHandleLayerLink;

  /// The objects supplied to the [CompositedTransformTarget] that wraps the
  /// location of end selection handle.
  final LayerLink endHandleLayerLink;

  /// {@template flutter.widgets.SelectionOverlay.selectionControls}
  /// Builds text selection handles and toolbar.
  /// {@endtemplate}
  final TextSelectionControls? selectionControls;

  /// {@template flutter.widgets.SelectionOverlay.selectionDelegate}
  /// The delegate for manipulating the current selection in the owning
  /// text field.
  /// {@endtemplate}
  @Deprecated(
    'Use `contextMenuBuilder` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  final TextSelectionDelegate? selectionDelegate;

  /// Determines the way that drag start behavior is handled.
  ///
  /// If set to [DragStartBehavior.start], handle drag behavior will
  /// begin at the position where the drag gesture won the arena. If set to
  /// [DragStartBehavior.down] it will begin at the position where a down
  /// event is first detected.
  ///
  /// In general, setting this to [DragStartBehavior.start] will make drag
  /// animation smoother and setting it to [DragStartBehavior.down] will make
  /// drag behavior feel slightly more reactive.
  ///
  /// By default, the drag start behavior is [DragStartBehavior.start].
  ///
  /// See also:
  ///
  ///  * [DragGestureRecognizer.dragStartBehavior], which gives an example for the different behaviors.
  final DragStartBehavior dragStartBehavior;

  /// {@template flutter.widgets.SelectionOverlay.onSelectionHandleTapped}
  /// A callback that's optionally invoked when a selection handle is tapped.
  ///
  /// The [TextSelectionControls.buildHandle] implementation the text field
  /// uses decides where the handle's tap "hotspot" is, or whether the
  /// selection handle supports tap gestures at all. For instance,
  /// [MaterialTextSelectionControls] calls [onSelectionHandleTapped] when the
  /// selection handle's "knob" is tapped, while
  /// [CupertinoTextSelectionControls] builds a handle that's not sufficiently
  /// large for tapping (as it's not meant to be tapped) so it does not call
  /// [onSelectionHandleTapped] even when tapped.
  /// {@endtemplate}
  // See https://github.com/flutter/flutter/issues/39376#issuecomment-848406415
  // for provenance.
  final VoidCallback? onSelectionHandleTapped;

  /// Maintains the status of the clipboard for determining if its contents can
  /// be pasted or not.
  ///
  /// Useful because the actual value of the clipboard can only be checked
  /// asynchronously (see [Clipboard.getData]).
  final ClipboardStatusNotifier? clipboardStatus;

  /// The location of where the toolbar should be drawn in relative to the
  /// location of [toolbarLayerLink].
  ///
  /// If this is null, the toolbar is drawn based on [selectionEndpoints] and
  /// the rect of render object of [context].
  ///
  /// This is useful for displaying toolbars at the mouse right-click locations
  /// in desktop devices.
  @Deprecated(
    'Use the `contextMenuBuilder` parameter in `showToolbar` instead. '
    'This feature was deprecated after v3.3.0-0.5.pre.',
  )
  Offset? get toolbarLocation => _toolbarLocation;
  Offset? _toolbarLocation;
  set toolbarLocation(Offset? value) {
    if (_toolbarLocation == value) {
      return;
    }
    _toolbarLocation = value;
    markNeedsBuild();
  }

  /// Controls the fade-in and fade-out animations for the toolbar and handles.
  static const Duration fadeDuration = Duration(milliseconds: 150);

  /// A pair of handles. If this is non-null, there are always 2, though the
  /// second is hidden when the selection is collapsed.
  ({OverlayEntry start, OverlayEntry end})? _handles;

  /// A copy/paste toolbar.
  OverlayEntry? _toolbar;

  // Manages the context menu. Not necessarily visible when non-null.
  final ContextMenuController _contextMenuController = ContextMenuController();

  final ContextMenuController _spellCheckToolbarController = ContextMenuController();

  /// {@template flutter.widgets.SelectionOverlay.showHandles}
  /// Builds the handles by inserting them into the [context]'s overlay.
  /// {@endtemplate}
  void showHandles() {
    if (_handles != null) {
      return;
    }

    final OverlayState overlay = Overlay.of(
      context,
      rootOverlay: true,
      debugRequiredFor: debugRequiredFor,
    );

    final CapturedThemes capturedThemes = InheritedTheme.capture(
      from: context,
      to: overlay.context,
    );

    _handles = (
      start: OverlayEntry(
        builder: (BuildContext context) {
          return capturedThemes.wrap(_buildStartHandle(context));
        },
      ),
      end: OverlayEntry(
        builder: (BuildContext context) {
          return capturedThemes.wrap(_buildEndHandle(context));
        },
      ),
    );
    overlay.insertAll(<OverlayEntry>[_handles!.start, _handles!.end]);
  }

  /// {@template flutter.widgets.SelectionOverlay.hideHandles}
  /// Destroys the handles by removing them from overlay.
  /// {@endtemplate}
  void hideHandles() {
    if (_handles != null) {
      _handles!.start.remove();
      _handles!.start.dispose();
      _handles!.end.remove();
      _handles!.end.dispose();
      _handles = null;
    }
  }

  /// {@template flutter.widgets.SelectionOverlay.showToolbar}
  /// Shows the toolbar by inserting it into the [context]'s overlay.
  /// {@endtemplate}
  void showToolbar({BuildContext? context, WidgetBuilder? contextMenuBuilder}) {
    if (contextMenuBuilder == null) {
      if (_toolbar != null) {
        return;
      }
      _toolbar = OverlayEntry(builder: _buildToolbar);
      Overlay.of(
        this.context,
        rootOverlay: true,
        debugRequiredFor: debugRequiredFor,
      ).insert(_toolbar!);
      return;
    }

    if (context == null) {
      return;
    }

    final RenderBox renderBox = context.findRenderObject()! as RenderBox;
    _contextMenuController.show(
      context: context,
      contextMenuBuilder: (BuildContext context) {
        return _SelectionToolbarWrapper(
          visibility: toolbarVisible,
          layerLink: toolbarLayerLink,
          offset: -renderBox.localToGlobal(Offset.zero),
          child: contextMenuBuilder(context),
        );
      },
    );
  }

  /// Shows toolbar with spell check suggestions of misspelled words that are
  /// available for click-and-replace.
  void showSpellCheckSuggestionsToolbar({BuildContext? context, required WidgetBuilder builder}) {
    if (context == null) {
      return;
    }

    final RenderBox renderBox = context.findRenderObject()! as RenderBox;
    _spellCheckToolbarController.show(
      context: context,
      contextMenuBuilder: (BuildContext context) {
        return _SelectionToolbarWrapper(
          layerLink: toolbarLayerLink,
          offset: -renderBox.localToGlobal(Offset.zero),
          child: builder(context),
        );
      },
    );
  }

  bool _buildScheduled = false;

  /// Rebuilds the selection toolbar or handles if they are present.
  void markNeedsBuild() {
    if (_handles == null && _toolbar == null) {
      return;
    }
    // If we are in build state, it will be too late to update visibility.
    // We will need to schedule the build in next frame.
    if (SchedulerBinding.instance.schedulerPhase == SchedulerPhase.persistentCallbacks) {
      if (_buildScheduled) {
        return;
      }
      _buildScheduled = true;
      SchedulerBinding.instance.addPostFrameCallback((Duration duration) {
        _buildScheduled = false;
        _handles?.start.markNeedsBuild();
        _handles?.end.markNeedsBuild();
        _toolbar?.markNeedsBuild();
        if (_contextMenuController.isShown) {
          _contextMenuController.markNeedsBuild();
        } else if (_spellCheckToolbarController.isShown) {
          _spellCheckToolbarController.markNeedsBuild();
        }
      }, debugLabel: 'SelectionOverlay.markNeedsBuild');
    } else {
      if (_handles != null) {
        _handles!.start.markNeedsBuild();
        _handles!.end.markNeedsBuild();
      }
      _toolbar?.markNeedsBuild();
      if (_contextMenuController.isShown) {
        _contextMenuController.markNeedsBuild();
      } else if (_spellCheckToolbarController.isShown) {
        _spellCheckToolbarController.markNeedsBuild();
      }
    }
  }

  /// {@template flutter.widgets.SelectionOverlay.hide}
  /// Hides the entire overlay including the toolbar and the handles.
  /// {@endtemplate}
  void hide() {
    _magnifierController.hide();
    hideHandles();
    if (_toolbar != null ||
        _contextMenuController.isShown ||
        _spellCheckToolbarController.isShown) {
      hideToolbar();
    }
  }

  /// {@template flutter.widgets.SelectionOverlay.hideToolbar}
  /// Hides the toolbar part of the overlay.
  ///
  /// To hide the whole overlay, see [hide].
  /// {@endtemplate}
  void hideToolbar() {
    _contextMenuController.remove();
    _spellCheckToolbarController.remove();
    if (_toolbar == null) {
      return;
    }
    _toolbar?.remove();
    _toolbar?.dispose();
    _toolbar = null;
  }

  /// {@template flutter.widgets.SelectionOverlay.dispose}
  /// Disposes this object and release resources.
  /// {@endtemplate}
  void dispose() {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }
    hide();
    _magnifierInfo.dispose();
  }

  Widget _buildStartHandle(BuildContext context) {
    final Widget handle;
    final TextSelectionControls? selectionControls = this.selectionControls;
    if (selectionControls == null ||
        (_startHandleType == TextSelectionHandleType.collapsed && _isDraggingEndHandle)) {
      // Hide the start handle when dragging the end handle and collapsing
      // the selection.
      handle = const SizedBox.shrink();
    } else {
      handle = _SelectionHandleOverlay(
        type: _startHandleType,
        handleLayerLink: startHandleLayerLink,
        onSelectionHandleTapped: onSelectionHandleTapped,
        onSelectionHandleDragStart: _handleStartHandleDragStart,
        onSelectionHandleDragUpdate: _handleStartHandleDragUpdate,
        onSelectionHandleDragEnd: _handleStartHandleDragEnd,
        selectionControls: selectionControls,
        visibility: startHandlesVisible,
        preferredLineHeight: _lineHeightAtStart,
        dragStartBehavior: dragStartBehavior,
      );
    }
    return TextFieldTapRegion(child: ExcludeSemantics(child: handle));
  }

  Widget _buildEndHandle(BuildContext context) {
    final Widget handle;
    final TextSelectionControls? selectionControls = this.selectionControls;
    if (selectionControls == null ||
        (_endHandleType == TextSelectionHandleType.collapsed && _isDraggingStartHandle) ||
        (_endHandleType == TextSelectionHandleType.collapsed &&
            !_isDraggingStartHandle &&
            !_isDraggingEndHandle)) {
      // Hide the end handle when dragging the start handle and collapsing the selection
      // or when the selection is collapsed and no handle is being dragged.
      handle = const SizedBox.shrink();
    } else {
      handle = _SelectionHandleOverlay(
        type: _endHandleType,
        handleLayerLink: endHandleLayerLink,
        onSelectionHandleTapped: onSelectionHandleTapped,
        onSelectionHandleDragStart: _handleEndHandleDragStart,
        onSelectionHandleDragUpdate: _handleEndHandleDragUpdate,
        onSelectionHandleDragEnd: _handleEndHandleDragEnd,
        selectionControls: selectionControls,
        visibility: endHandlesVisible,
        preferredLineHeight: _lineHeightAtEnd,
        dragStartBehavior: dragStartBehavior,
      );
    }
    return TextFieldTapRegion(child: ExcludeSemantics(child: handle));
  }

  // Build the toolbar via TextSelectionControls.
  Widget _buildToolbar(BuildContext context) {
    if (selectionControls == null) {
      return const SizedBox.shrink();
    }
    assert(
      selectionDelegate != null,
      'If not using contextMenuBuilder, must pass selectionDelegate.',
    );

    final RenderBox renderBox = this.context.findRenderObject()! as RenderBox;

    final Rect editingRegion = Rect.fromPoints(
      renderBox.localToGlobal(Offset.zero),
      renderBox.localToGlobal(renderBox.size.bottomRight(Offset.zero)),
    );

    final bool isMultiline =
        selectionEndpoints.last.point.dy - selectionEndpoints.first.point.dy > lineHeightAtEnd / 2;

    // If the selected text spans more than 1 line, horizontally center the toolbar.
    // Derived from both iOS and Android.
    final double midX =
        isMultiline
            ? editingRegion.width / 2
            : (selectionEndpoints.first.point.dx + selectionEndpoints.last.point.dx) / 2;

    final Offset midpoint = Offset(
      midX,
      // The y-coordinate won't be made use of most likely.
      selectionEndpoints.first.point.dy - lineHeightAtStart,
    );

    return _SelectionToolbarWrapper(
      visibility: toolbarVisible,
      layerLink: toolbarLayerLink,
      offset: -editingRegion.topLeft,
      child: Builder(
        builder: (BuildContext context) {
          return selectionControls!.buildToolbar(
            context,
            editingRegion,
            lineHeightAtStart,
            midpoint,
            selectionEndpoints,
            selectionDelegate!,
            clipboardStatus,
            toolbarLocation,
          );
        },
      ),
    );
  }

  /// {@template flutter.widgets.SelectionOverlay.updateMagnifier}
  /// Update the current magnifier with new selection data, so the magnifier
  /// can respond accordingly.
  ///
  /// If the magnifier is not shown, this still updates the magnifier position
  /// because the magnifier may have hidden itself and is looking for a cue to reshow
  /// itself.
  ///
  /// If there is no magnifier in the overlay, this does nothing.
  /// {@endtemplate}
  void updateMagnifier(MagnifierInfo magnifierInfo) {
    if (_magnifierController.overlayEntry == null) {
      return;
    }

    _magnifierInfo.value = magnifierInfo;
  }
}

// TODO(justinmc): Currently this fades in but not out on all platforms. It
// should follow the correct fading behavior for the current platform, then be
// made public and de-duplicated with widgets/selectable_region.dart.
// https://github.com/flutter/flutter/issues/107732
// Wrap the given child in the widgets common to both contextMenuBuilder and
// TextSelectionControls.buildToolbar.
class _SelectionToolbarWrapper extends StatefulWidget {
  const _SelectionToolbarWrapper({
    this.visibility,
    required this.layerLink,
    required this.offset,
    required this.child,
  });

  final Widget child;
  final Offset offset;
  final LayerLink layerLink;
  final ValueListenable<bool>? visibility;

  @override
  State<_SelectionToolbarWrapper> createState() => _SelectionToolbarWrapperState();
}

class _SelectionToolbarWrapperState extends State<_SelectionToolbarWrapper>
    with SingleTickerProviderStateMixin {
  late AnimationController _controller;
  Animation<double> get _opacity => _controller.view;

  @override
  void initState() {
    super.initState();

    _controller = AnimationController(duration: SelectionOverlay.fadeDuration, vsync: this);

    _toolbarVisibilityChanged();
    widget.visibility?.addListener(_toolbarVisibilityChanged);
  }

  @override
  void didUpdateWidget(_SelectionToolbarWrapper oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.visibility == widget.visibility) {
      return;
    }
    oldWidget.visibility?.removeListener(_toolbarVisibilityChanged);
    _toolbarVisibilityChanged();
    widget.visibility?.addListener(_toolbarVisibilityChanged);
  }

  @override
  void dispose() {
    widget.visibility?.removeListener(_toolbarVisibilityChanged);
    _controller.dispose();
    super.dispose();
  }

  void _toolbarVisibilityChanged() {
    if (widget.visibility?.value ?? true) {
      _controller.forward();
    } else {
      _controller.reverse();
    }
  }

  @override
  Widget build(BuildContext context) {
    return TextFieldTapRegion(
      child: Directionality(
        textDirection: Directionality.of(this.context),
        child: FadeTransition(
          opacity: _opacity,
          child: CompositedTransformFollower(
            link: widget.layerLink,
            showWhenUnlinked: false,
            offset: widget.offset,
            child: widget.child,
          ),
        ),
      ),
    );
  }
}

/// This widget represents a single draggable selection handle.
class _SelectionHandleOverlay extends StatefulWidget {
  /// Create selection overlay.
  const _SelectionHandleOverlay({
    required this.type,
    required this.handleLayerLink,
    this.onSelectionHandleTapped,
    this.onSelectionHandleDragStart,
    this.onSelectionHandleDragUpdate,
    this.onSelectionHandleDragEnd,
    required this.selectionControls,
    this.visibility,
    required this.preferredLineHeight,
    this.dragStartBehavior = DragStartBehavior.start,
  });

  final LayerLink handleLayerLink;
  final VoidCallback? onSelectionHandleTapped;
  final ValueChanged<DragStartDetails>? onSelectionHandleDragStart;
  final ValueChanged<DragUpdateDetails>? onSelectionHandleDragUpdate;
  final ValueChanged<DragEndDetails>? onSelectionHandleDragEnd;
  final TextSelectionControls selectionControls;
  final ValueListenable<bool>? visibility;
  final double preferredLineHeight;
  final TextSelectionHandleType type;
  final DragStartBehavior dragStartBehavior;

  @override
  State<_SelectionHandleOverlay> createState() => _SelectionHandleOverlayState();
}

class _SelectionHandleOverlayState extends State<_SelectionHandleOverlay>
    with SingleTickerProviderStateMixin {
  late AnimationController _controller;
  Animation<double> get _opacity => _controller.view;

  @override
  void initState() {
    super.initState();

    _controller = AnimationController(duration: SelectionOverlay.fadeDuration, vsync: this);

    _handleVisibilityChanged();
    widget.visibility?.addListener(_handleVisibilityChanged);
  }

  void _handleVisibilityChanged() {
    if (widget.visibility?.value ?? true) {
      _controller.forward();
    } else {
      _controller.reverse();
    }
  }

  /// Returns the bounding [Rect] of the text selection handle in local
  /// coordinates.
  ///
  /// When interacting with a text selection handle through a touch event, the
  /// interactive area should be at least [kMinInteractiveDimension] square,
  /// which this method does not consider.
  Rect _getHandleRect(TextSelectionHandleType type, double preferredLineHeight) {
    final Size handleSize = widget.selectionControls.getHandleSize(preferredLineHeight);
    return Rect.fromLTWH(0.0, 0.0, handleSize.width, handleSize.height);
  }

  @override
  void didUpdateWidget(_SelectionHandleOverlay oldWidget) {
    super.didUpdateWidget(oldWidget);
    oldWidget.visibility?.removeListener(_handleVisibilityChanged);
    _handleVisibilityChanged();
    widget.visibility?.addListener(_handleVisibilityChanged);
  }

  @override
  void dispose() {
    widget.visibility?.removeListener(_handleVisibilityChanged);
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final Rect handleRect = _getHandleRect(widget.type, widget.preferredLineHeight);

    // Make sure the GestureDetector is big enough to be easily interactive.
    final Rect interactiveRect = handleRect.expandToInclude(
      Rect.fromCircle(center: handleRect.center, radius: kMinInteractiveDimension / 2),
    );
    final RelativeRect padding = RelativeRect.fromLTRB(
      math.max((interactiveRect.width - handleRect.width) / 2, 0),
      math.max((interactiveRect.height - handleRect.height) / 2, 0),
      math.max((interactiveRect.width - handleRect.width) / 2, 0),
      math.max((interactiveRect.height - handleRect.height) / 2, 0),
    );

    final Offset handleAnchor = widget.selectionControls.getHandleAnchor(
      widget.type,
      widget.preferredLineHeight,
    );

    // Make sure a drag is eagerly accepted. This is used on iOS to match the
    // behavior where a drag directly on a collapse handle will always win against
    // other drag gestures.
    final bool eagerlyAcceptDragWhenCollapsed =
        widget.type == TextSelectionHandleType.collapsed &&
        defaultTargetPlatform == TargetPlatform.iOS;

    return CompositedTransformFollower(
      link: widget.handleLayerLink,
      // Put the handle's anchor point on the leader's anchor point.
      offset: -handleAnchor - Offset(padding.left, padding.top),
      showWhenUnlinked: false,
      child: FadeTransition(
        opacity: _opacity,
        child: SizedBox(
          width: interactiveRect.width,
          height: interactiveRect.height,
          child: Align(
            alignment: Alignment.topLeft,
            child: RawGestureDetector(
              behavior: HitTestBehavior.translucent,
              gestures: <Type, GestureRecognizerFactory>{
                PanGestureRecognizer: GestureRecognizerFactoryWithHandlers<PanGestureRecognizer>(
                  () => PanGestureRecognizer(
                    debugOwner: this,
                    // Mouse events select the text and do not drag the cursor.
                    supportedDevices: <PointerDeviceKind>{
                      PointerDeviceKind.touch,
                      PointerDeviceKind.stylus,
                      PointerDeviceKind.unknown,
                    },
                  ),
                  (PanGestureRecognizer instance) {
                    instance
                      ..dragStartBehavior = widget.dragStartBehavior
                      ..gestureSettings =
                          eagerlyAcceptDragWhenCollapsed
                              ? const DeviceGestureSettings(touchSlop: 1.0)
                              : null
                      ..onStart = widget.onSelectionHandleDragStart
                      ..onUpdate = widget.onSelectionHandleDragUpdate
                      ..onEnd = widget.onSelectionHandleDragEnd;
                  },
                ),
              },
              child: Padding(
                padding: EdgeInsets.only(
                  left: padding.left,
                  top: padding.top,
                  right: padding.right,
                  bottom: padding.bottom,
                ),
                child: widget.selectionControls.buildHandle(
                  context,
                  widget.type,
                  widget.preferredLineHeight,
                  widget.onSelectionHandleTapped,
                ),
              ),
            ),
          ),
        ),
      ),
    );
  }
}

/// Delegate interface for the [TextSelectionGestureDetectorBuilder].
///
/// The interface is usually implemented by the [State] of text field
/// implementations wrapping [EditableText], so that they can use a
/// [TextSelectionGestureDetectorBuilder] to build a
/// [TextSelectionGestureDetector] for their [EditableText]. The delegate
/// provides the builder with information about the current state of the text
/// field. Based on that information, the builder adds the correct gesture
/// handlers to the gesture detector.
///
/// See also:
///
///  * [TextField], which implements this delegate for the Material text field.
///  * [CupertinoTextField], which implements this delegate for the Cupertino
///    text field.
abstract class TextSelectionGestureDetectorBuilderDelegate {
  /// [GlobalKey] to the [EditableText] for which the
  /// [TextSelectionGestureDetectorBuilder] will build a [TextSelectionGestureDetector].
  GlobalKey<EditableTextState> get editableTextKey;

  /// Whether the text field should respond to force presses.
  bool get forcePressEnabled;

  /// Whether the user may select text in the text field.
  bool get selectionEnabled;
}

/// Builds a [TextSelectionGestureDetector] to wrap an [EditableText].
///
/// The class implements sensible defaults for many user interactions
/// with an [EditableText] (see the documentation of the various gesture handler
/// methods, e.g. [onTapDown], [onForcePressStart], etc.). Subclasses of
/// [TextSelectionGestureDetectorBuilder] can change the behavior performed in
/// responds to these gesture events by overriding the corresponding handler
/// methods of this class.
///
/// The resulting [TextSelectionGestureDetector] to wrap an [EditableText] is
/// obtained by calling [buildGestureDetector].
///
/// A [TextSelectionGestureDetectorBuilder] must be provided a
/// [TextSelectionGestureDetectorBuilderDelegate], from which information about
/// the [EditableText] may be obtained. Typically, the [State] of the widget
/// that builds the [EditableText] implements this interface, and then passes
/// itself as the [delegate].
///
/// See also:
///
///  * [TextField], which uses a subclass to implement the Material-specific
///    gesture logic of an [EditableText].
///  * [CupertinoTextField], which uses a subclass to implement the
///    Cupertino-specific gesture logic of an [EditableText].
class TextSelectionGestureDetectorBuilder {
  /// Creates a [TextSelectionGestureDetectorBuilder].
  TextSelectionGestureDetectorBuilder({required this.delegate});

  /// The delegate for this [TextSelectionGestureDetectorBuilder].
  ///
  /// The delegate provides the builder with information about what actions can
  /// currently be performed on the text field. Based on this, the builder adds
  /// the correct gesture handlers to the gesture detector.
  ///
  /// Typically implemented by a [State] of a widget that builds an
  /// [EditableText].
  @protected
  final TextSelectionGestureDetectorBuilderDelegate delegate;

  // Shows the magnifier on supported platforms at the given offset, currently
  // only Android and iOS.
  void _showMagnifierIfSupportedByPlatform(Offset positionToShow) {
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
      case TargetPlatform.iOS:
        editableText.showMagnifier(positionToShow);
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.macOS:
      case TargetPlatform.windows:
    }
  }

  // Hides the magnifier on supported platforms, currently only Android and iOS.
  void _hideMagnifierIfSupportedByPlatform() {
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
      case TargetPlatform.iOS:
        editableText.hideMagnifier();
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.macOS:
      case TargetPlatform.windows:
    }
  }

  /// Returns true if lastSecondaryTapDownPosition was on selection.
  bool get _lastSecondaryTapWasOnSelection {
    assert(renderEditable.lastSecondaryTapDownPosition != null);
    if (renderEditable.selection == null) {
      return false;
    }

    final TextPosition textPosition = renderEditable.getPositionForPoint(
      renderEditable.lastSecondaryTapDownPosition!,
    );

    return renderEditable.selection!.start <= textPosition.offset &&
        renderEditable.selection!.end >= textPosition.offset;
  }

  bool _positionWasOnSelectionExclusive(TextPosition textPosition) {
    final TextSelection? selection = renderEditable.selection;
    if (selection == null) {
      return false;
    }

    return selection.start < textPosition.offset && selection.end > textPosition.offset;
  }

  bool _positionWasOnSelectionInclusive(TextPosition textPosition) {
    final TextSelection? selection = renderEditable.selection;
    if (selection == null) {
      return false;
    }

    return selection.start <= textPosition.offset && selection.end >= textPosition.offset;
  }

  // Expand the selection to the given global position.
  //
  // Either base or extent will be moved to the last tapped position, whichever
  // is closest. The selection will never shrink or pivot, only grow.
  //
  // If fromSelection is given, will expand from that selection instead of the
  // current selection in renderEditable.
  //
  // See also:
  //
  //   * [_extendSelection], which is similar but pivots the selection around
  //     the base.
  void _expandSelection(
    Offset offset,
    SelectionChangedCause cause, [
    TextSelection? fromSelection,
  ]) {
    assert(renderEditable.selection?.baseOffset != null);

    final TextPosition tappedPosition = renderEditable.getPositionForPoint(offset);
    final TextSelection selection = fromSelection ?? renderEditable.selection!;
    final bool baseIsCloser =
        (tappedPosition.offset - selection.baseOffset).abs() <
        (tappedPosition.offset - selection.extentOffset).abs();
    final TextSelection nextSelection = selection.copyWith(
      baseOffset: baseIsCloser ? selection.extentOffset : selection.baseOffset,
      extentOffset: tappedPosition.offset,
    );

    editableText.userUpdateTextEditingValue(
      editableText.textEditingValue.copyWith(selection: nextSelection),
      cause,
    );
  }

  // Extend the selection to the given global position.
  //
  // Holds the base in place and moves the extent.
  //
  // See also:
  //
  //   * [_expandSelection], which is similar but always increases the size of
  //     the selection.
  void _extendSelection(Offset offset, SelectionChangedCause cause) {
    assert(renderEditable.selection?.baseOffset != null);

    final TextPosition tappedPosition = renderEditable.getPositionForPoint(offset);
    final TextSelection selection = renderEditable.selection!;
    final TextSelection nextSelection = selection.copyWith(extentOffset: tappedPosition.offset);

    editableText.userUpdateTextEditingValue(
      editableText.textEditingValue.copyWith(selection: nextSelection),
      cause,
    );
  }

  /// Whether to show the selection toolbar.
  ///
  /// It is based on the signal source when a [onTapDown] is called. This getter
  /// will return true if current [onTapDown] event is triggered by a touch or
  /// a stylus.
  bool get shouldShowSelectionToolbar => _shouldShowSelectionToolbar;
  bool _shouldShowSelectionToolbar = true;

  /// The [State] of the [EditableText] for which the builder will provide a
  /// [TextSelectionGestureDetector].
  @protected
  EditableTextState get editableText => delegate.editableTextKey.currentState!;

  /// The [RenderObject] of the [EditableText] for which the builder will
  /// provide a [TextSelectionGestureDetector].
  @protected
  RenderEditable get renderEditable => editableText.renderEditable;

  /// Whether the Shift key was pressed when the most recent [PointerDownEvent]
  /// was tracked by the [BaseTapAndDragGestureRecognizer].
  bool _isShiftPressed = false;

  /// The viewport offset pixels of any [Scrollable] containing the
  /// [RenderEditable] at the last drag start.
  double _dragStartScrollOffset = 0.0;

  /// The viewport offset pixels of the [RenderEditable] at the last drag start.
  double _dragStartViewportOffset = 0.0;

  double get _scrollPosition {
    final ScrollableState? scrollableState =
        delegate.editableTextKey.currentContext == null
            ? null
            : Scrollable.maybeOf(delegate.editableTextKey.currentContext!);
    return scrollableState == null ? 0.0 : scrollableState.position.pixels;
  }

  AxisDirection? get _scrollDirection {
    final ScrollableState? scrollableState =
        delegate.editableTextKey.currentContext == null
            ? null
            : Scrollable.maybeOf(delegate.editableTextKey.currentContext!);
    return scrollableState?.axisDirection;
  }

  // For a shift + tap + drag gesture, the TextSelection at the point of the
  // tap. Mac uses this value to reset to the original selection when an
  // inversion of the base and offset happens.
  TextSelection? _dragStartSelection;

  // For iOS long press behavior when the field is not focused. iOS uses this value
  // to determine if a long press began on a field that was not focused.
  //
  // If the field was not focused when the long press began, a long press will select
  // the word and a long press move will select word-by-word. If the field was
  // focused, the cursor moves to the long press position.
  bool _longPressStartedWithoutFocus = false;

  /// Handler for [TextSelectionGestureDetector.onTapTrackStart].
  ///
  /// See also:
  ///
  ///  * [TextSelectionGestureDetector.onTapTrackStart], which triggers this
  ///    callback.
  @protected
  void onTapTrackStart() {
    _isShiftPressed =
        HardwareKeyboard.instance.logicalKeysPressed.intersection(<LogicalKeyboardKey>{
          LogicalKeyboardKey.shiftLeft,
          LogicalKeyboardKey.shiftRight,
        }).isNotEmpty;
  }

  /// Handler for [TextSelectionGestureDetector.onTapTrackReset].
  ///
  /// See also:
  ///
  ///  * [TextSelectionGestureDetector.onTapTrackReset], which triggers this
  ///    callback.
  @protected
  void onTapTrackReset() {
    _isShiftPressed = false;
  }

  /// Handler for [TextSelectionGestureDetector.onTapDown].
  ///
  /// By default, it forwards the tap to [RenderEditable.handleTapDown] and sets
  /// [shouldShowSelectionToolbar] to true if the tap was initiated by a finger or stylus.
  ///
  /// See also:
  ///
  ///  * [TextSelectionGestureDetector.onTapDown], which triggers this callback.
  @protected
  void onTapDown(TapDragDownDetails details) {
    if (!delegate.selectionEnabled) {
      return;
    }

    // TODO(Renzo-Olivares): Migrate text selection gestures away from saving state
    // in renderEditable. The gesture callbacks can use the details objects directly
    // in callbacks variants that provide them [TapGestureRecognizer.onSecondaryTap]
    // vs [TapGestureRecognizer.onSecondaryTapUp] instead of having to track state in
    // renderEditable. When this migration is complete we should remove this hack.
    // See https://github.com/flutter/flutter/issues/115130.
    renderEditable.handleTapDown(TapDownDetails(globalPosition: details.globalPosition));
    // The selection overlay should only be shown when the user is interacting
    // through a touch screen (via either a finger or a stylus). A mouse shouldn't
    // trigger the selection overlay.
    // For backwards-compatibility, we treat a null kind the same as touch.
    final PointerDeviceKind? kind = details.kind;
    // TODO(justinmc): Should a desktop platform show its selection toolbar when
    // receiving a tap event?  Say a Windows device with a touchscreen.
    // https://github.com/flutter/flutter/issues/106586
    _shouldShowSelectionToolbar =
        kind == null || kind == PointerDeviceKind.touch || kind == PointerDeviceKind.stylus;

    // It is impossible to extend the selection when the shift key is pressed, if the
    // renderEditable.selection is invalid.
    final bool isShiftPressedValid =
        _isShiftPressed && renderEditable.selection?.baseOffset != null;
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
        if (editableText.widget.stylusHandwritingEnabled) {
          final bool stylusEnabled = switch (kind) {
            PointerDeviceKind.stylus ||
            PointerDeviceKind.invertedStylus => editableText.widget.stylusHandwritingEnabled,
            _ => false,
          };
          if (stylusEnabled) {
            Scribe.isFeatureAvailable().then((bool isAvailable) {
              if (isAvailable) {
                renderEditable.selectPosition(cause: SelectionChangedCause.scribble);
                Scribe.startStylusHandwriting();
              }
            });
          }
        }
      case TargetPlatform.fuchsia:
      case TargetPlatform.iOS:
        // On mobile platforms the selection is set on tap up.
        break;
      case TargetPlatform.macOS:
        editableText.hideToolbar();
        // On macOS, a shift-tapped unfocused field expands from 0, not from the
        // previous selection.
        if (isShiftPressedValid) {
          final TextSelection? fromSelection =
              renderEditable.hasFocus ? null : const TextSelection.collapsed(offset: 0);
          _expandSelection(details.globalPosition, SelectionChangedCause.tap, fromSelection);
          return;
        }
        // On macOS, a tap/click places the selection in a precise position.
        // This differs from iOS/iPadOS, where if the gesture is done by a touch
        // then the selection moves to the closest word edge, instead of a
        // precise position.
        renderEditable.selectPosition(cause: SelectionChangedCause.tap);
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        editableText.hideToolbar();
        if (isShiftPressedValid) {
          _extendSelection(details.globalPosition, SelectionChangedCause.tap);
          return;
        }
        renderEditable.selectPosition(cause: SelectionChangedCause.tap);
    }
  }

  /// Handler for [TextSelectionGestureDetector.onForcePressStart].
  ///
  /// By default, it selects the word at the position of the force press,
  /// if selection is enabled.
  ///
  /// This callback is only applicable when force press is enabled.
  ///
  /// See also:
  ///
  ///  * [TextSelectionGestureDetector.onForcePressStart], which triggers this
  ///    callback.
  @protected
  void onForcePressStart(ForcePressDetails details) {
    assert(delegate.forcePressEnabled);
    _shouldShowSelectionToolbar = true;
    if (!delegate.selectionEnabled) {
      return;
    }
    renderEditable.selectWordsInRange(
      from: details.globalPosition,
      cause: SelectionChangedCause.forcePress,
    );
    editableText.showToolbar();
  }

  /// Handler for [TextSelectionGestureDetector.onForcePressEnd].
  ///
  /// By default, it selects words in the range specified in [details] and shows
  /// toolbar if it is necessary.
  ///
  /// This callback is only applicable when force press is enabled.
  ///
  /// See also:
  ///
  ///  * [TextSelectionGestureDetector.onForcePressEnd], which triggers this
  ///    callback.
  @protected
  void onForcePressEnd(ForcePressDetails details) {
    assert(delegate.forcePressEnabled);
    renderEditable.selectWordsInRange(
      from: details.globalPosition,
      cause: SelectionChangedCause.forcePress,
    );
    if (shouldShowSelectionToolbar) {
      editableText.showToolbar();
    }
  }

  /// Whether the provided [onUserTap] callback should be dispatched on every
  /// tap or only non-consecutive taps.
  ///
  /// Defaults to false.
  @protected
  bool get onUserTapAlwaysCalled => false;

  /// Handler for [TextSelectionGestureDetector.onUserTap].
  ///
  /// By default, it serves as placeholder to enable subclass override.
  ///
  /// See also:
  ///
  ///  * [TextSelectionGestureDetector.onUserTap], which triggers this
  ///    callback.
  ///  * [TextSelectionGestureDetector.onUserTapAlwaysCalled], which controls
  ///     whether this callback is called only on the first tap in a series
  ///     of taps.
  @protected
  void onUserTap() {
    /* Subclass should override this method if needed. */
  }

  /// Handler for [TextSelectionGestureDetector.onSingleTapUp].
  ///
  /// By default, it selects word edge if selection is enabled.
  ///
  /// See also:
  ///
  ///  * [TextSelectionGestureDetector.onSingleTapUp], which triggers
  ///    this callback.
  @protected
  void onSingleTapUp(TapDragUpDetails details) {
    if (!delegate.selectionEnabled) {
      editableText.requestKeyboard();
      return;
    }
    // It is impossible to extend the selection when the shift key is pressed, if the
    // renderEditable.selection is invalid.
    final bool isShiftPressedValid =
        _isShiftPressed && renderEditable.selection?.baseOffset != null;
    switch (defaultTargetPlatform) {
      case TargetPlatform.linux:
      case TargetPlatform.macOS:
      case TargetPlatform.windows:
        break;
      // On desktop platforms the selection is set on tap down.
      case TargetPlatform.android:
        editableText.hideToolbar(false);
        if (isShiftPressedValid) {
          _extendSelection(details.globalPosition, SelectionChangedCause.tap);
          return;
        }
        renderEditable.selectPosition(cause: SelectionChangedCause.tap);
        editableText.showSpellCheckSuggestionsToolbar();
      case TargetPlatform.fuchsia:
        editableText.hideToolbar(false);
        if (isShiftPressedValid) {
          _extendSelection(details.globalPosition, SelectionChangedCause.tap);
          return;
        }
        renderEditable.selectPosition(cause: SelectionChangedCause.tap);
      case TargetPlatform.iOS:
        if (isShiftPressedValid) {
          // On iOS, a shift-tapped unfocused field expands from 0, not from
          // the previous selection.
          final TextSelection? fromSelection =
              renderEditable.hasFocus ? null : const TextSelection.collapsed(offset: 0);
          _expandSelection(details.globalPosition, SelectionChangedCause.tap, fromSelection);
          return;
        }
        switch (details.kind) {
          case PointerDeviceKind.mouse:
          case PointerDeviceKind.trackpad:
          case PointerDeviceKind.stylus:
          case PointerDeviceKind.invertedStylus:
            // TODO(camsim99): Determine spell check toolbar behavior in these cases:
            // https://github.com/flutter/flutter/issues/119573.
            // Precise devices should place the cursor at a precise position if the
            // word at the text position is not misspelled.
            renderEditable.selectPosition(cause: SelectionChangedCause.tap);
          case PointerDeviceKind.touch:
          case PointerDeviceKind.unknown:
            // If the word that was tapped is misspelled, select the word and show the spell check suggestions
            // toolbar once. If additional taps are made on a misspelled word, toggle the toolbar. If the word
            // is not misspelled, default to the following behavior:
            //
            // Toggle the toolbar when the tap is exclusively within the bounds of a non-collapsed `previousSelection`,
            // and the editable is focused.
            //
            // Toggle the toolbar if the `previousSelection` is collapsed, the tap is on the selection, the
            // TextAffinity remains the same, the editable field is not read only, and the editable is focused.
            // The TextAffinity is important when the cursor is on the boundary of a line wrap, if the affinity
            // is different (i.e. it is downstream), the selection should move to the following line and not toggle
            // the toolbar.
            //
            // Selects the word edge closest to the tap when the editable is not focused, or if the tap was neither exclusively
            // or inclusively on `previousSelection`. If the selection remains the same after selecting the word edge, then we
            // toggle the toolbar, if the editable field is not read only. If the selection changes then we hide the toolbar.
            final TextSelection previousSelection =
                renderEditable.selection ?? editableText.textEditingValue.selection;
            final TextPosition textPosition = renderEditable.getPositionForPoint(
              details.globalPosition,
            );
            final bool isAffinityTheSame = textPosition.affinity == previousSelection.affinity;
            final bool wordAtCursorIndexIsMisspelled =
                editableText.findSuggestionSpanAtCursorIndex(textPosition.offset) != null;

            if (wordAtCursorIndexIsMisspelled) {
              renderEditable.selectWord(cause: SelectionChangedCause.tap);
              if (previousSelection != editableText.textEditingValue.selection) {
                editableText.showSpellCheckSuggestionsToolbar();
              } else {
                editableText.toggleToolbar(false);
              }
            } else if (((_positionWasOnSelectionExclusive(textPosition) &&
                        !previousSelection.isCollapsed) ||
                    (_positionWasOnSelectionInclusive(textPosition) &&
                        previousSelection.isCollapsed &&
                        isAffinityTheSame &&
                        !renderEditable.readOnly)) &&
                renderEditable.hasFocus) {
              editableText.toggleToolbar(false);
            } else {
              renderEditable.selectWordEdge(cause: SelectionChangedCause.tap);
              if (previousSelection == editableText.textEditingValue.selection &&
                  renderEditable.hasFocus &&
                  !renderEditable.readOnly) {
                editableText.toggleToolbar(false);
              } else {
                editableText.hideToolbar(false);
              }
            }
        }
    }
    editableText.requestKeyboard();
  }

  /// Handler for [TextSelectionGestureDetector.onSingleTapCancel].
  ///
  /// By default, it serves as placeholder to enable subclass override.
  ///
  /// See also:
  ///
  ///  * [TextSelectionGestureDetector.onSingleTapCancel], which triggers
  ///    this callback.
  @protected
  void onSingleTapCancel() {
    /* Subclass should override this method if needed. */
  }

  /// Handler for [TextSelectionGestureDetector.onSingleLongTapStart].
  ///
  /// By default, it selects text position specified in [details] if selection
  /// is enabled.
  ///
  /// See also:
  ///
  ///  * [TextSelectionGestureDetector.onSingleLongTapStart], which triggers
  ///    this callback.
  @protected
  void onSingleLongTapStart(LongPressStartDetails details) {
    if (!delegate.selectionEnabled) {
      return;
    }
    switch (defaultTargetPlatform) {
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        if (!renderEditable.hasFocus) {
          _longPressStartedWithoutFocus = true;
          renderEditable.selectWord(cause: SelectionChangedCause.longPress);
        } else if (renderEditable.readOnly) {
          renderEditable.selectWord(cause: SelectionChangedCause.longPress);
          if (editableText.context.mounted) {
            Feedback.forLongPress(editableText.context);
          }
        } else {
          renderEditable.selectPositionAt(
            from: details.globalPosition,
            cause: SelectionChangedCause.longPress,
          );
          // Show the floating cursor.
          final RawFloatingCursorPoint cursorPoint = RawFloatingCursorPoint(
            state: FloatingCursorDragState.Start,
            startLocation: (
              renderEditable.globalToLocal(details.globalPosition),
              TextPosition(
                offset: editableText.textEditingValue.selection.baseOffset,
                affinity: editableText.textEditingValue.selection.affinity,
              ),
            ),
            offset: Offset.zero,
          );
          editableText.updateFloatingCursor(cursorPoint);
        }
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        renderEditable.selectWord(cause: SelectionChangedCause.longPress);
        if (editableText.context.mounted) {
          Feedback.forLongPress(editableText.context);
        }
    }

    _showMagnifierIfSupportedByPlatform(details.globalPosition);

    _dragStartViewportOffset = renderEditable.offset.pixels;
    _dragStartScrollOffset = _scrollPosition;
  }

  /// Handler for [TextSelectionGestureDetector.onSingleLongTapMoveUpdate].
  ///
  /// By default, it updates the selection location specified in [details] if
  /// selection is enabled.
  ///
  /// See also:
  ///
  ///  * [TextSelectionGestureDetector.onSingleLongTapMoveUpdate], which
  ///    triggers this callback.
  @protected
  void onSingleLongTapMoveUpdate(LongPressMoveUpdateDetails details) {
    if (!delegate.selectionEnabled) {
      return;
    }
    // Adjust the drag start offset for possible viewport offset changes.
    final Offset editableOffset =
        renderEditable.maxLines == 1
            ? Offset(renderEditable.offset.pixels - _dragStartViewportOffset, 0.0)
            : Offset(0.0, renderEditable.offset.pixels - _dragStartViewportOffset);
    final Offset scrollableOffset = switch (axisDirectionToAxis(
      _scrollDirection ?? AxisDirection.left,
    )) {
      Axis.horizontal => Offset(_scrollPosition - _dragStartScrollOffset, 0.0),
      Axis.vertical => Offset(0.0, _scrollPosition - _dragStartScrollOffset),
    };
    switch (defaultTargetPlatform) {
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        if (_longPressStartedWithoutFocus || renderEditable.readOnly) {
          renderEditable.selectWordsInRange(
            from:
                details.globalPosition -
                details.offsetFromOrigin -
                editableOffset -
                scrollableOffset,
            to: details.globalPosition,
            cause: SelectionChangedCause.longPress,
          );
        } else {
          renderEditable.selectPositionAt(
            from: details.globalPosition,
            cause: SelectionChangedCause.longPress,
          );
          // Update the floating cursor.
          final RawFloatingCursorPoint cursorPoint = RawFloatingCursorPoint(
            state: FloatingCursorDragState.Update,
            offset: details.offsetFromOrigin,
          );
          editableText.updateFloatingCursor(cursorPoint);
        }
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        renderEditable.selectWordsInRange(
          from:
              details.globalPosition - details.offsetFromOrigin - editableOffset - scrollableOffset,
          to: details.globalPosition,
          cause: SelectionChangedCause.longPress,
        );
    }

    _showMagnifierIfSupportedByPlatform(details.globalPosition);
  }

  /// Handler for [TextSelectionGestureDetector.onSingleLongTapEnd].
  ///
  /// By default, it shows toolbar if necessary.
  ///
  /// See also:
  ///
  ///  * [TextSelectionGestureDetector.onSingleLongTapEnd], which triggers this
  ///    callback.
  @protected
  void onSingleLongTapEnd(LongPressEndDetails details) {
    _hideMagnifierIfSupportedByPlatform();
    if (shouldShowSelectionToolbar) {
      editableText.showToolbar();
    }
    _longPressStartedWithoutFocus = false;
    _dragStartViewportOffset = 0.0;
    _dragStartScrollOffset = 0.0;
    if (defaultTargetPlatform == TargetPlatform.iOS &&
        delegate.selectionEnabled &&
        editableText.textEditingValue.selection.isCollapsed) {
      // Update the floating cursor.
      final RawFloatingCursorPoint cursorPoint = RawFloatingCursorPoint(
        state: FloatingCursorDragState.End,
      );
      editableText.updateFloatingCursor(cursorPoint);
    }
  }

  /// Handler for [TextSelectionGestureDetector.onSecondaryTap].
  ///
  /// By default, selects the word if possible and shows the toolbar.
  @protected
  void onSecondaryTap() {
    if (!delegate.selectionEnabled) {
      return;
    }
    switch (defaultTargetPlatform) {
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        if (!_lastSecondaryTapWasOnSelection || !renderEditable.hasFocus) {
          renderEditable.selectWord(cause: SelectionChangedCause.tap);
        }
        if (shouldShowSelectionToolbar) {
          editableText.hideToolbar();
          editableText.showToolbar();
        }
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        if (!renderEditable.hasFocus) {
          renderEditable.selectPosition(cause: SelectionChangedCause.tap);
        }
        editableText.toggleToolbar();
    }
  }

  /// Handler for [TextSelectionGestureDetector.onSecondaryTapDown].
  ///
  /// See also:
  ///
  ///  * [TextSelectionGestureDetector.onSecondaryTapDown], which triggers this
  ///    callback.
  ///  * [onSecondaryTap], which is typically called after this.
  @protected
  void onSecondaryTapDown(TapDownDetails details) {
    // TODO(Renzo-Olivares): Migrate text selection gestures away from saving state
    // in renderEditable. The gesture callbacks can use the details objects directly
    // in callbacks variants that provide them [TapGestureRecognizer.onSecondaryTap]
    // vs [TapGestureRecognizer.onSecondaryTapUp] instead of having to track state in
    // renderEditable. When this migration is complete we should remove this hack.
    // See https://github.com/flutter/flutter/issues/115130.
    renderEditable.handleSecondaryTapDown(TapDownDetails(globalPosition: details.globalPosition));
    _shouldShowSelectionToolbar = true;
  }

  /// Handler for [TextSelectionGestureDetector.onDoubleTapDown].
  ///
  /// By default, it selects a word through [RenderEditable.selectWord] if
  /// selectionEnabled and shows toolbar if necessary.
  ///
  /// See also:
  ///
  ///  * [TextSelectionGestureDetector.onDoubleTapDown], which triggers this
  ///    callback.
  @protected
  void onDoubleTapDown(TapDragDownDetails details) {
    if (delegate.selectionEnabled) {
      renderEditable.selectWord(cause: SelectionChangedCause.doubleTap);
      if (shouldShowSelectionToolbar) {
        editableText.showToolbar();
      }
    }
  }

  // Selects the set of paragraphs in a document that intersect a given range of
  // global positions.
  void _selectParagraphsInRange({required Offset from, Offset? to, SelectionChangedCause? cause}) {
    final TextBoundary paragraphBoundary = ParagraphBoundary(editableText.textEditingValue.text);
    _selectTextBoundariesInRange(boundary: paragraphBoundary, from: from, to: to, cause: cause);
  }

  // Selects the set of lines in a document that intersect a given range of
  // global positions.
  void _selectLinesInRange({required Offset from, Offset? to, SelectionChangedCause? cause}) {
    final TextBoundary lineBoundary = LineBoundary(renderEditable);
    _selectTextBoundariesInRange(boundary: lineBoundary, from: from, to: to, cause: cause);
  }

  // Returns the location of a text boundary at `extent`. When `extent` is at
  // the end of the text, returns the previous text boundary's location.
  TextRange _moveToTextBoundary(TextPosition extent, TextBoundary textBoundary) {
    assert(extent.offset >= 0);
    // Use extent.offset - 1 when `extent` is at the end of the text to retrieve
    // the previous text boundary's location.
    final int start =
        textBoundary.getLeadingTextBoundaryAt(
          extent.offset == editableText.textEditingValue.text.length
              ? extent.offset - 1
              : extent.offset,
        ) ??
        0;
    final int end =
        textBoundary.getTrailingTextBoundaryAt(extent.offset) ??
        editableText.textEditingValue.text.length;
    return TextRange(start: start, end: end);
  }

  // Selects the set of text boundaries in a document that intersect a given
  // range of global positions.
  //
  // The set of text boundaries selected are not strictly bounded by the range
  // of global positions.
  //
  // The first and last endpoints of the selection will always be at the
  // beginning and end of a text boundary respectively.
  void _selectTextBoundariesInRange({
    required TextBoundary boundary,
    required Offset from,
    Offset? to,
    SelectionChangedCause? cause,
  }) {
    final TextPosition fromPosition = renderEditable.getPositionForPoint(from);
    final TextRange fromRange = _moveToTextBoundary(fromPosition, boundary);
    final TextPosition toPosition =
        to == null ? fromPosition : renderEditable.getPositionForPoint(to);
    final TextRange toRange =
        toPosition == fromPosition ? fromRange : _moveToTextBoundary(toPosition, boundary);
    final bool isFromBoundaryBeforeToBoundary = fromRange.start < toRange.end;

    final TextSelection newSelection =
        isFromBoundaryBeforeToBoundary
            ? TextSelection(baseOffset: fromRange.start, extentOffset: toRange.end)
            : TextSelection(baseOffset: fromRange.end, extentOffset: toRange.start);

    editableText.userUpdateTextEditingValue(
      editableText.textEditingValue.copyWith(selection: newSelection),
      cause,
    );
  }

  /// Handler for [TextSelectionGestureDetector.onTripleTapDown].
  ///
  /// By default, it selects a paragraph if
  /// [TextSelectionGestureDetectorBuilderDelegate.selectionEnabled] is true
  /// and shows the toolbar if necessary.
  ///
  /// See also:
  ///
  ///  * [TextSelectionGestureDetector.onTripleTapDown], which triggers this
  ///    callback.
  @protected
  void onTripleTapDown(TapDragDownDetails details) {
    if (!delegate.selectionEnabled) {
      return;
    }
    if (renderEditable.maxLines == 1) {
      editableText.selectAll(SelectionChangedCause.tap);
    } else {
      switch (defaultTargetPlatform) {
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          _selectParagraphsInRange(from: details.globalPosition, cause: SelectionChangedCause.tap);
        case TargetPlatform.linux:
          _selectLinesInRange(from: details.globalPosition, cause: SelectionChangedCause.tap);
      }
    }
    if (shouldShowSelectionToolbar) {
      editableText.showToolbar();
    }
  }

  /// Handler for [TextSelectionGestureDetector.onDragSelectionStart].
  ///
  /// By default, it selects a text position specified in [details].
  ///
  /// See also:
  ///
  ///  * [TextSelectionGestureDetector.onDragSelectionStart], which triggers
  ///    this callback.
  @protected
  void onDragSelectionStart(TapDragStartDetails details) {
    if (!delegate.selectionEnabled) {
      return;
    }
    final PointerDeviceKind? kind = details.kind;
    _shouldShowSelectionToolbar =
        kind == null || kind == PointerDeviceKind.touch || kind == PointerDeviceKind.stylus;

    _dragStartSelection = renderEditable.selection;
    _dragStartScrollOffset = _scrollPosition;
    _dragStartViewportOffset = renderEditable.offset.pixels;

    if (_TextSelectionGestureDetectorState._getEffectiveConsecutiveTapCount(
          details.consecutiveTapCount,
        ) >
        1) {
      // Do not set the selection on a consecutive tap and drag.
      return;
    }

    if (_isShiftPressed && renderEditable.selection != null && renderEditable.selection!.isValid) {
      switch (defaultTargetPlatform) {
        case TargetPlatform.iOS:
        case TargetPlatform.macOS:
          _expandSelection(details.globalPosition, SelectionChangedCause.drag);
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          _extendSelection(details.globalPosition, SelectionChangedCause.drag);
      }
    } else {
      switch (defaultTargetPlatform) {
        case TargetPlatform.iOS:
          switch (details.kind) {
            case PointerDeviceKind.mouse:
            case PointerDeviceKind.trackpad:
              renderEditable.selectPositionAt(
                from: details.globalPosition,
                cause: SelectionChangedCause.drag,
              );
            case PointerDeviceKind.stylus:
            case PointerDeviceKind.invertedStylus:
            case PointerDeviceKind.touch:
            case PointerDeviceKind.unknown:
            case null:
          }
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
          switch (details.kind) {
            case PointerDeviceKind.mouse:
            case PointerDeviceKind.trackpad:
              renderEditable.selectPositionAt(
                from: details.globalPosition,
                cause: SelectionChangedCause.drag,
              );
            case PointerDeviceKind.stylus:
            case PointerDeviceKind.invertedStylus:
            case PointerDeviceKind.touch:
            case PointerDeviceKind.unknown:
              // For Android, Fuchsia, and iOS platforms, a touch drag
              // does not initiate unless the editable has focus.
              if (renderEditable.hasFocus) {
                renderEditable.selectPositionAt(
                  from: details.globalPosition,
                  cause: SelectionChangedCause.drag,
                );
                _showMagnifierIfSupportedByPlatform(details.globalPosition);
              }
            case null:
          }
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          renderEditable.selectPositionAt(
            from: details.globalPosition,
            cause: SelectionChangedCause.drag,
          );
      }
    }
  }

  /// Handler for [TextSelectionGestureDetector.onDragSelectionUpdate].
  ///
  /// By default, it updates the selection location specified in the provided
  /// details objects.
  ///
  /// See also:
  ///
  ///  * [TextSelectionGestureDetector.onDragSelectionUpdate], which triggers
  ///    this callback./lib/src/material/text_field.dart
  @protected
  void onDragSelectionUpdate(TapDragUpdateDetails details) {
    if (!delegate.selectionEnabled) {
      return;
    }

    if (!_isShiftPressed) {
      // Adjust the drag start offset for possible viewport offset changes.
      final Offset editableOffset =
          renderEditable.maxLines == 1
              ? Offset(renderEditable.offset.pixels - _dragStartViewportOffset, 0.0)
              : Offset(0.0, renderEditable.offset.pixels - _dragStartViewportOffset);
      final Offset scrollableOffset = switch (axisDirectionToAxis(
        _scrollDirection ?? AxisDirection.left,
      )) {
        Axis.horizontal => Offset(_scrollPosition - _dragStartScrollOffset, 0.0),
        Axis.vertical => Offset(0.0, _scrollPosition - _dragStartScrollOffset),
      };
      final Offset dragStartGlobalPosition = details.globalPosition - details.offsetFromOrigin;

      // Select word by word.
      if (_TextSelectionGestureDetectorState._getEffectiveConsecutiveTapCount(
            details.consecutiveTapCount,
          ) ==
          2) {
        renderEditable.selectWordsInRange(
          from: dragStartGlobalPosition - editableOffset - scrollableOffset,
          to: details.globalPosition,
          cause: SelectionChangedCause.drag,
        );

        switch (details.kind) {
          case PointerDeviceKind.stylus:
          case PointerDeviceKind.invertedStylus:
          case PointerDeviceKind.touch:
          case PointerDeviceKind.unknown:
            return _showMagnifierIfSupportedByPlatform(details.globalPosition);
          case PointerDeviceKind.mouse:
          case PointerDeviceKind.trackpad:
          case null:
            return;
        }
      }

      // Select paragraph-by-paragraph.
      if (_TextSelectionGestureDetectorState._getEffectiveConsecutiveTapCount(
            details.consecutiveTapCount,
          ) ==
          3) {
        switch (defaultTargetPlatform) {
          case TargetPlatform.android:
          case TargetPlatform.fuchsia:
          case TargetPlatform.iOS:
            switch (details.kind) {
              case PointerDeviceKind.mouse:
              case PointerDeviceKind.trackpad:
                return _selectParagraphsInRange(
                  from: dragStartGlobalPosition - editableOffset - scrollableOffset,
                  to: details.globalPosition,
                  cause: SelectionChangedCause.drag,
                );
              case PointerDeviceKind.stylus:
              case PointerDeviceKind.invertedStylus:
              case PointerDeviceKind.touch:
              case PointerDeviceKind.unknown:
              case null:
                // Triple tap to drag is not present on these platforms when using
                // non-precise pointer devices at the moment.
                break;
            }
            return;
          case TargetPlatform.linux:
            return _selectLinesInRange(
              from: dragStartGlobalPosition - editableOffset - scrollableOffset,
              to: details.globalPosition,
              cause: SelectionChangedCause.drag,
            );
          case TargetPlatform.windows:
          case TargetPlatform.macOS:
            return _selectParagraphsInRange(
              from: dragStartGlobalPosition - editableOffset - scrollableOffset,
              to: details.globalPosition,
              cause: SelectionChangedCause.drag,
            );
        }
      }

      switch (defaultTargetPlatform) {
        case TargetPlatform.iOS:
          // With a mouse device, a drag should select the range from the origin of the drag
          // to the current position of the drag.
          //
          // With a touch device, nothing should happen.
          switch (details.kind) {
            case PointerDeviceKind.mouse:
            case PointerDeviceKind.trackpad:
              return renderEditable.selectPositionAt(
                from: dragStartGlobalPosition - editableOffset - scrollableOffset,
                to: details.globalPosition,
                cause: SelectionChangedCause.drag,
              );
            case PointerDeviceKind.stylus:
            case PointerDeviceKind.invertedStylus:
            case PointerDeviceKind.touch:
            case PointerDeviceKind.unknown:
            case null:
              break;
          }
          return;
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
          // With a precise pointer device, such as a mouse, trackpad, or stylus,
          // the drag will select the text spanning the origin of the drag to the end of the drag.
          // With a touch device, the cursor should move with the drag.
          switch (details.kind) {
            case PointerDeviceKind.mouse:
            case PointerDeviceKind.trackpad:
            case PointerDeviceKind.stylus:
            case PointerDeviceKind.invertedStylus:
              return renderEditable.selectPositionAt(
                from: dragStartGlobalPosition - editableOffset - scrollableOffset,
                to: details.globalPosition,
                cause: SelectionChangedCause.drag,
              );
            case PointerDeviceKind.touch:
            case PointerDeviceKind.unknown:
              if (renderEditable.hasFocus) {
                renderEditable.selectPositionAt(
                  from: details.globalPosition,
                  cause: SelectionChangedCause.drag,
                );
                return _showMagnifierIfSupportedByPlatform(details.globalPosition);
              }
            case null:
              break;
          }
          return;
        case TargetPlatform.macOS:
        case TargetPlatform.linux:
        case TargetPlatform.windows:
          return renderEditable.selectPositionAt(
            from: dragStartGlobalPosition - editableOffset - scrollableOffset,
            to: details.globalPosition,
            cause: SelectionChangedCause.drag,
          );
      }
    }

    if (_dragStartSelection!.isCollapsed ||
        (defaultTargetPlatform != TargetPlatform.iOS &&
            defaultTargetPlatform != TargetPlatform.macOS)) {
      return _extendSelection(details.globalPosition, SelectionChangedCause.drag);
    }

    // If the drag inverts the selection, Mac and iOS revert to the initial
    // selection.
    final TextSelection selection = editableText.textEditingValue.selection;
    final TextPosition nextExtent = renderEditable.getPositionForPoint(details.globalPosition);
    final bool isShiftTapDragSelectionForward =
        _dragStartSelection!.baseOffset < _dragStartSelection!.extentOffset;
    final bool isInverted =
        isShiftTapDragSelectionForward
            ? nextExtent.offset < _dragStartSelection!.baseOffset
            : nextExtent.offset > _dragStartSelection!.baseOffset;
    if (isInverted && selection.baseOffset == _dragStartSelection!.baseOffset) {
      editableText.userUpdateTextEditingValue(
        editableText.textEditingValue.copyWith(
          selection: TextSelection(
            baseOffset: _dragStartSelection!.extentOffset,
            extentOffset: nextExtent.offset,
          ),
        ),
        SelectionChangedCause.drag,
      );
    } else if (!isInverted &&
        nextExtent.offset != _dragStartSelection!.baseOffset &&
        selection.baseOffset != _dragStartSelection!.baseOffset) {
      editableText.userUpdateTextEditingValue(
        editableText.textEditingValue.copyWith(
          selection: TextSelection(
            baseOffset: _dragStartSelection!.baseOffset,
            extentOffset: nextExtent.offset,
          ),
        ),
        SelectionChangedCause.drag,
      );
    } else {
      _extendSelection(details.globalPosition, SelectionChangedCause.drag);
    }
  }

  /// Handler for [TextSelectionGestureDetector.onDragSelectionEnd].
  ///
  /// By default, it cleans up the state used for handling certain
  /// built-in behaviors.
  ///
  /// See also:
  ///
  ///  * [TextSelectionGestureDetector.onDragSelectionEnd], which triggers this
  ///    callback.
  @protected
  void onDragSelectionEnd(TapDragEndDetails details) {
    if (_shouldShowSelectionToolbar &&
        _TextSelectionGestureDetectorState._getEffectiveConsecutiveTapCount(
              details.consecutiveTapCount,
            ) ==
            2) {
      editableText.showToolbar();
    }

    if (_isShiftPressed) {
      _dragStartSelection = null;
    }

    _hideMagnifierIfSupportedByPlatform();
  }

  /// Returns a [TextSelectionGestureDetector] configured with the handlers
  /// provided by this builder.
  ///
  /// The [child] or its subtree should contain an [EditableText] whose key is
  /// the [GlobalKey] provided by the [delegate]'s
  /// [TextSelectionGestureDetectorBuilderDelegate.editableTextKey].
  Widget buildGestureDetector({Key? key, HitTestBehavior? behavior, required Widget child}) {
    return TextSelectionGestureDetector(
      key: key,
      onTapTrackStart: onTapTrackStart,
      onTapTrackReset: onTapTrackReset,
      onTapDown: onTapDown,
      onForcePressStart: delegate.forcePressEnabled ? onForcePressStart : null,
      onForcePressEnd: delegate.forcePressEnabled ? onForcePressEnd : null,
      onSecondaryTap: onSecondaryTap,
      onSecondaryTapDown: onSecondaryTapDown,
      onSingleTapUp: onSingleTapUp,
      onSingleTapCancel: onSingleTapCancel,
      onUserTap: onUserTap,
      onSingleLongTapStart: onSingleLongTapStart,
      onSingleLongTapMoveUpdate: onSingleLongTapMoveUpdate,
      onSingleLongTapEnd: onSingleLongTapEnd,
      onDoubleTapDown: onDoubleTapDown,
      onTripleTapDown: onTripleTapDown,
      onDragSelectionStart: onDragSelectionStart,
      onDragSelectionUpdate: onDragSelectionUpdate,
      onDragSelectionEnd: onDragSelectionEnd,
      onUserTapAlwaysCalled: onUserTapAlwaysCalled,
      behavior: behavior,
      child: child,
    );
  }
}

/// A gesture detector to respond to non-exclusive event chains for a text field.
///
/// An ordinary [GestureDetector] configured to handle events like tap and
/// double tap will only recognize one or the other. This widget detects both:
/// the first tap and then any subsequent taps that occurs within a time limit
/// after the first.
///
/// See also:
///
///  * [TextField], a Material text field which uses this gesture detector.
///  * [CupertinoTextField], a Cupertino text field which uses this gesture
///    detector.
class TextSelectionGestureDetector extends StatefulWidget {
  /// Create a [TextSelectionGestureDetector].
  ///
  /// Multiple callbacks can be called for one sequence of input gesture.
  const TextSelectionGestureDetector({
    super.key,
    this.onTapTrackStart,
    this.onTapTrackReset,
    this.onTapDown,
    this.onForcePressStart,
    this.onForcePressEnd,
    this.onSecondaryTap,
    this.onSecondaryTapDown,
    this.onSingleTapUp,
    this.onSingleTapCancel,
    this.onUserTap,
    this.onSingleLongTapStart,
    this.onSingleLongTapMoveUpdate,
    this.onSingleLongTapEnd,
    this.onDoubleTapDown,
    this.onTripleTapDown,
    this.onDragSelectionStart,
    this.onDragSelectionUpdate,
    this.onDragSelectionEnd,
    this.onUserTapAlwaysCalled = false,
    this.behavior,
    required this.child,
  });

  /// {@template flutter.gestures.selectionrecognizers.TextSelectionGestureDetector.onTapTrackStart}
  /// Callback used to indicate that a tap tracking has started upon
  /// a [PointerDownEvent].
  /// {@endtemplate}
  final VoidCallback? onTapTrackStart;

  /// {@template flutter.gestures.selectionrecognizers.TextSelectionGestureDetector.onTapTrackReset}
  /// Callback used to indicate that a tap tracking has been reset which
  /// happens on the next [PointerDownEvent] after the timer between two taps
  /// elapses, the recognizer loses the arena, the gesture is cancelled or
  /// the recognizer is disposed of.
  /// {@endtemplate}
  final VoidCallback? onTapTrackReset;

  /// Called for every tap down including every tap down that's part of a
  /// double click or a long press, except touches that include enough movement
  /// to not qualify as taps (e.g. pans and flings).
  final GestureTapDragDownCallback? onTapDown;

  /// Called when a pointer has tapped down and the force of the pointer has
  /// just become greater than [ForcePressGestureRecognizer.startPressure].
  final GestureForcePressStartCallback? onForcePressStart;

  /// Called when a pointer that had previously triggered [onForcePressStart] is
  /// lifted off the screen.
  final GestureForcePressEndCallback? onForcePressEnd;

  /// Called for a tap event with the secondary mouse button.
  final GestureTapCallback? onSecondaryTap;

  /// Called for a tap down event with the secondary mouse button.
  final GestureTapDownCallback? onSecondaryTapDown;

  /// Called for the first tap in a series of taps, consecutive taps do not call
  /// this method.
  ///
  /// For example, if the detector was configured with [onTapDown] and
  /// [onDoubleTapDown], three quick taps would be recognized as a single tap
  /// down, followed by a tap up, then a double tap down, followed by a single tap down.
  final GestureTapDragUpCallback? onSingleTapUp;

  /// Called for each touch that becomes recognized as a gesture that is not a
  /// short tap, such as a long tap or drag. It is called at the moment when
  /// another gesture from the touch is recognized.
  final GestureCancelCallback? onSingleTapCancel;

  /// Called for the first tap in a series of taps when [onUserTapAlwaysCalled] is
  /// disabled, which is the default behavior.
  ///
  /// When [onUserTapAlwaysCalled] is enabled, this is called for every tap,
  /// including consecutive taps.
  final GestureTapCallback? onUserTap;

  /// Called for a single long tap that's sustained for longer than
  /// [kLongPressTimeout] but not necessarily lifted. Not called for a
  /// double-tap-hold, which calls [onDoubleTapDown] instead.
  final GestureLongPressStartCallback? onSingleLongTapStart;

  /// Called after [onSingleLongTapStart] when the pointer is dragged.
  final GestureLongPressMoveUpdateCallback? onSingleLongTapMoveUpdate;

  /// Called after [onSingleLongTapStart] when the pointer is lifted.
  final GestureLongPressEndCallback? onSingleLongTapEnd;

  /// Called after a momentary hold or a short tap that is close in space and
  /// time (within [kDoubleTapTimeout]) to a previous short tap.
  final GestureTapDragDownCallback? onDoubleTapDown;

  /// Called after a momentary hold or a short tap that is close in space and
  /// time (within [kDoubleTapTimeout]) to a previous double-tap.
  final GestureTapDragDownCallback? onTripleTapDown;

  /// Called when a mouse starts dragging to select text.
  final GestureTapDragStartCallback? onDragSelectionStart;

  /// Called repeatedly as a mouse moves while dragging.
  final GestureTapDragUpdateCallback? onDragSelectionUpdate;

  /// Called when a mouse that was previously dragging is released.
  final GestureTapDragEndCallback? onDragSelectionEnd;

  /// Whether [onUserTap] will be called for all taps including consecutive taps.
  ///
  /// Defaults to false, so [onUserTap] is only called for each distinct tap.
  final bool onUserTapAlwaysCalled;

  /// How this gesture detector should behave during hit testing.
  ///
  /// This defaults to [HitTestBehavior.deferToChild].
  final HitTestBehavior? behavior;

  /// Child below this widget.
  final Widget child;

  @override
  State<StatefulWidget> createState() => _TextSelectionGestureDetectorState();
}

class _TextSelectionGestureDetectorState extends State<TextSelectionGestureDetector> {
  // Converts the details.consecutiveTapCount from a TapAndDrag*Details object,
  // which can grow to be infinitely large, to a value between 1 and 3. The value
  // that the raw count is converted to is based on the default observed behavior
  // on the native platforms.
  //
  // This method should be used in all instances when details.consecutiveTapCount
  // would be used.
  static int _getEffectiveConsecutiveTapCount(int rawCount) {
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
        // From observation, these platform's reset their tap count to 0 when
        // the number of consecutive taps exceeds 3. For example on Debian Linux
        // with GTK, when going past a triple click, on the fourth click the
        // selection is moved to the precise click position, on the fifth click
        // the word at the position is selected, and on the sixth click the
        // paragraph at the position is selected.
        return rawCount <= 3 ? rawCount : (rawCount % 3 == 0 ? 3 : rawCount % 3);
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        // From observation, these platform's either hold their tap count at 3.
        // For example on macOS, when going past a triple click, the selection
        // should be retained at the paragraph that was first selected on triple
        // click.
        return math.min(rawCount, 3);
      case TargetPlatform.windows:
        // From observation, this platform's consecutive tap actions alternate
        // between double click and triple click actions. For example, after a
        // triple click has selected a paragraph, on the next click the word at
        // the clicked position will be selected, and on the next click the
        // paragraph at the position is selected.
        return rawCount < 2 ? rawCount : 2 + rawCount % 2;
    }
  }

  void _handleTapTrackStart() {
    widget.onTapTrackStart?.call();
  }

  void _handleTapTrackReset() {
    widget.onTapTrackReset?.call();
  }

  // The down handler is force-run on success of a single tap and optimistically
  // run before a long press success.
  void _handleTapDown(TapDragDownDetails details) {
    widget.onTapDown?.call(details);
    // This isn't detected as a double tap gesture in the gesture recognizer
    // because it's 2 single taps, each of which may do different things depending
    // on whether it's a single tap, the first tap of a double tap, the second
    // tap held down, a clean double tap etc.
    if (_getEffectiveConsecutiveTapCount(details.consecutiveTapCount) == 2) {
      return widget.onDoubleTapDown?.call(details);
    }

    if (_getEffectiveConsecutiveTapCount(details.consecutiveTapCount) == 3) {
      return widget.onTripleTapDown?.call(details);
    }
  }

  void _handleTapUp(TapDragUpDetails details) {
    if (_getEffectiveConsecutiveTapCount(details.consecutiveTapCount) == 1) {
      widget.onSingleTapUp?.call(details);
      widget.onUserTap?.call();
    } else if (widget.onUserTapAlwaysCalled) {
      widget.onUserTap?.call();
    }
  }

  void _handleTapCancel() {
    widget.onSingleTapCancel?.call();
  }

  void _handleDragStart(TapDragStartDetails details) {
    widget.onDragSelectionStart?.call(details);
  }

  void _handleDragUpdate(TapDragUpdateDetails details) {
    widget.onDragSelectionUpdate?.call(details);
  }

  void _handleDragEnd(TapDragEndDetails details) {
    widget.onDragSelectionEnd?.call(details);
  }

  void _forcePressStarted(ForcePressDetails details) {
    widget.onForcePressStart?.call(details);
  }

  void _forcePressEnded(ForcePressDetails details) {
    widget.onForcePressEnd?.call(details);
  }

  void _handleLongPressStart(LongPressStartDetails details) {
    if (widget.onSingleLongTapStart != null) {
      widget.onSingleLongTapStart!(details);
    }
  }

  void _handleLongPressMoveUpdate(LongPressMoveUpdateDetails details) {
    if (widget.onSingleLongTapMoveUpdate != null) {
      widget.onSingleLongTapMoveUpdate!(details);
    }
  }

  void _handleLongPressEnd(LongPressEndDetails details) {
    if (widget.onSingleLongTapEnd != null) {
      widget.onSingleLongTapEnd!(details);
    }
  }

  @override
  Widget build(BuildContext context) {
    final Map<Type, GestureRecognizerFactory> gestures = <Type, GestureRecognizerFactory>{};

    gestures[TapGestureRecognizer] = GestureRecognizerFactoryWithHandlers<TapGestureRecognizer>(
      () => TapGestureRecognizer(debugOwner: this),
      (TapGestureRecognizer instance) {
        instance
          ..onSecondaryTap = widget.onSecondaryTap
          ..onSecondaryTapDown = widget.onSecondaryTapDown;
      },
    );

    if (widget.onSingleLongTapStart != null ||
        widget.onSingleLongTapMoveUpdate != null ||
        widget.onSingleLongTapEnd != null) {
      gestures[LongPressGestureRecognizer] =
          GestureRecognizerFactoryWithHandlers<LongPressGestureRecognizer>(
            () => LongPressGestureRecognizer(
              debugOwner: this,
              supportedDevices: <PointerDeviceKind>{PointerDeviceKind.touch},
            ),
            (LongPressGestureRecognizer instance) {
              instance
                ..onLongPressStart = _handleLongPressStart
                ..onLongPressMoveUpdate = _handleLongPressMoveUpdate
                ..onLongPressEnd = _handleLongPressEnd;
            },
          );
    }

    if (widget.onDragSelectionStart != null ||
        widget.onDragSelectionUpdate != null ||
        widget.onDragSelectionEnd != null) {
      switch (defaultTargetPlatform) {
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.iOS:
          gestures[TapAndHorizontalDragGestureRecognizer] =
              GestureRecognizerFactoryWithHandlers<TapAndHorizontalDragGestureRecognizer>(
                () => TapAndHorizontalDragGestureRecognizer(debugOwner: this),
                (TapAndHorizontalDragGestureRecognizer instance) {
                  instance
                    // Text selection should start from the position of the first pointer
                    // down event.
                    ..dragStartBehavior = DragStartBehavior.down
                    ..eagerVictoryOnDrag = defaultTargetPlatform != TargetPlatform.iOS
                    ..onTapTrackStart = _handleTapTrackStart
                    ..onTapTrackReset = _handleTapTrackReset
                    ..onTapDown = _handleTapDown
                    ..onDragStart = _handleDragStart
                    ..onDragUpdate = _handleDragUpdate
                    ..onDragEnd = _handleDragEnd
                    ..onTapUp = _handleTapUp
                    ..onCancel = _handleTapCancel;
                },
              );
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          gestures[TapAndPanGestureRecognizer] =
              GestureRecognizerFactoryWithHandlers<TapAndPanGestureRecognizer>(
                () => TapAndPanGestureRecognizer(debugOwner: this),
                (TapAndPanGestureRecognizer instance) {
                  instance
                    // Text selection should start from the position of the first pointer
                    // down event.
                    ..dragStartBehavior = DragStartBehavior.down
                    ..onTapTrackStart = _handleTapTrackStart
                    ..onTapTrackReset = _handleTapTrackReset
                    ..onTapDown = _handleTapDown
                    ..onDragStart = _handleDragStart
                    ..onDragUpdate = _handleDragUpdate
                    ..onDragEnd = _handleDragEnd
                    ..onTapUp = _handleTapUp
                    ..onCancel = _handleTapCancel;
                },
              );
      }
    }

    if (widget.onForcePressStart != null || widget.onForcePressEnd != null) {
      gestures[ForcePressGestureRecognizer] =
          GestureRecognizerFactoryWithHandlers<ForcePressGestureRecognizer>(
            () => ForcePressGestureRecognizer(debugOwner: this),
            (ForcePressGestureRecognizer instance) {
              instance
                ..onStart = widget.onForcePressStart != null ? _forcePressStarted : null
                ..onEnd = widget.onForcePressEnd != null ? _forcePressEnded : null;
            },
          );
    }

    return RawGestureDetector(
      gestures: gestures,
      excludeFromSemantics: true,
      behavior: widget.behavior,
      child: widget.child,
    );
  }
}

/// A [ValueNotifier] whose [value] indicates whether the current contents of
/// the clipboard can be pasted.
///
/// The contents of the clipboard can only be read asynchronously, via
/// [Clipboard.getData], so this maintains a value that can be used
/// synchronously. Call [update] to asynchronously update value if needed.
class ClipboardStatusNotifier extends ValueNotifier<ClipboardStatus> with WidgetsBindingObserver {
  /// Create a new ClipboardStatusNotifier.
  ClipboardStatusNotifier({ClipboardStatus value = ClipboardStatus.unknown}) : super(value);

  bool _disposed = false;

  /// Check the [Clipboard] and update [value] if needed.
  Future<void> update() async {
    if (_disposed) {
      return;
    }

    final bool hasStrings;
    try {
      hasStrings = await Clipboard.hasStrings();
    } catch (exception, stack) {
      FlutterError.reportError(
        FlutterErrorDetails(
          exception: exception,
          stack: stack,
          library: 'widget library',
          context: ErrorDescription('while checking if the clipboard has strings'),
        ),
      );
      // In the case of an error from the Clipboard API, set the value to
      // unknown so that it will try to update again later.
      if (_disposed) {
        return;
      }
      value = ClipboardStatus.unknown;
      return;
    }
    final ClipboardStatus nextStatus =
        hasStrings ? ClipboardStatus.pasteable : ClipboardStatus.notPasteable;

    if (_disposed) {
      return;
    }
    value = nextStatus;
  }

  @override
  void addListener(VoidCallback listener) {
    if (!hasListeners) {
      WidgetsBinding.instance.addObserver(this);
    }
    if (value == ClipboardStatus.unknown) {
      update();
    }
    super.addListener(listener);
  }

  @override
  void removeListener(VoidCallback listener) {
    super.removeListener(listener);
    if (!_disposed && !hasListeners) {
      WidgetsBinding.instance.removeObserver(this);
    }
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    switch (state) {
      case AppLifecycleState.resumed:
        update();
      case AppLifecycleState.detached:
      case AppLifecycleState.inactive:
      case AppLifecycleState.hidden:
      case AppLifecycleState.paused:
        // Nothing to do.
        break;
    }
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    _disposed = true;
    super.dispose();
  }
}

/// An enumeration of the status of the content on the user's clipboard.
enum ClipboardStatus {
  /// The clipboard content can be pasted, such as a String of nonzero length.
  pasteable,

  /// The status of the clipboard is unknown. Since getting clipboard data is
  /// asynchronous (see [Clipboard.getData]), this status often exists while
  /// waiting to receive the clipboard contents for the first time.
  unknown,

  /// The content on the clipboard is not pasteable, such as when it is empty.
  notPasteable,
}

/// A [ValueNotifier] whose [value] indicates whether the current device supports the Live Text
/// (OCR) function.
///
/// See also:
///  * [LiveText], where the availability of Live Text input can be obtained.
///  * [LiveTextInputStatus], an enumeration that indicates whether the current device is available
///                           for Live Text input.
///
/// Call [update] to asynchronously update [value] if needed.
class LiveTextInputStatusNotifier extends ValueNotifier<LiveTextInputStatus>
    with WidgetsBindingObserver {
  /// Create a new LiveTextStatusNotifier.
  LiveTextInputStatusNotifier({LiveTextInputStatus value = LiveTextInputStatus.unknown})
    : super(value);

  bool _disposed = false;

  /// Check the [LiveTextInputStatus] and update [value] if needed.
  Future<void> update() async {
    if (_disposed) {
      return;
    }

    final bool isLiveTextInputEnabled;
    try {
      isLiveTextInputEnabled = await LiveText.isLiveTextInputAvailable();
    } catch (exception, stack) {
      FlutterError.reportError(
        FlutterErrorDetails(
          exception: exception,
          stack: stack,
          library: 'widget library',
          context: ErrorDescription('while checking the availability of Live Text input'),
        ),
      );
      // In the case of an error from the Live Text API, set the value to
      // unknown so that it will try to update again later.
      if (_disposed || value == LiveTextInputStatus.unknown) {
        return;
      }
      value = LiveTextInputStatus.unknown;
      return;
    }

    final LiveTextInputStatus nextStatus =
        isLiveTextInputEnabled ? LiveTextInputStatus.enabled : LiveTextInputStatus.disabled;

    if (_disposed || nextStatus == value) {
      return;
    }
    value = nextStatus;
  }

  @override
  void addListener(VoidCallback listener) {
    if (!hasListeners) {
      WidgetsBinding.instance.addObserver(this);
    }
    if (value == LiveTextInputStatus.unknown) {
      update();
    }
    super.addListener(listener);
  }

  @override
  void removeListener(VoidCallback listener) {
    super.removeListener(listener);
    if (!_disposed && !hasListeners) {
      WidgetsBinding.instance.removeObserver(this);
    }
  }

  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    switch (state) {
      case AppLifecycleState.resumed:
        update();
      case AppLifecycleState.detached:
      case AppLifecycleState.inactive:
      case AppLifecycleState.paused:
      case AppLifecycleState.hidden:
      // Nothing to do.
    }
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    _disposed = true;
    super.dispose();
  }
}

/// An enumeration that indicates whether the current device is available for Live Text input.
///
/// See also:
///  * [LiveText], where the availability of Live Text input can be obtained.
enum LiveTextInputStatus {
  /// This device supports Live Text input currently.
  enabled,

  /// The status of the Live Text input is unknown. Since getting the Live Text input availability
  /// is asynchronous (see [LiveText.isLiveTextInputAvailable]), this status often exists while
  /// waiting to receive the status value for the first time.
  unknown,

  /// The current device doesn't support Live Text input.
  disabled,
}

// TODO(justinmc): Deprecate this after TextSelectionControls.buildToolbar is
// deleted, when users should migrate back to TextSelectionControls.buildHandle.
// See https://github.com/flutter/flutter/pull/124262
/// [TextSelectionControls] that specifically do not manage the toolbar in order
/// to leave that to [EditableText.contextMenuBuilder].
mixin TextSelectionHandleControls on TextSelectionControls {
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
  ) => const SizedBox.shrink();

  @override
  bool canCut(TextSelectionDelegate delegate) => false;

  @override
  bool canCopy(TextSelectionDelegate delegate) => false;

  @override
  bool canPaste(TextSelectionDelegate delegate) => false;

  @override
  bool canSelectAll(TextSelectionDelegate delegate) => false;

  @override
  void handleCut(TextSelectionDelegate delegate, [ClipboardStatusNotifier? clipboardStatus]) {}

  @override
  void handleCopy(TextSelectionDelegate delegate, [ClipboardStatusNotifier? clipboardStatus]) {}

  @override
  Future<void> handlePaste(TextSelectionDelegate delegate) async {}

  @override
  void handleSelectAll(TextSelectionDelegate delegate) {}
}
