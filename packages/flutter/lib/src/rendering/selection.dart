// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/gestures.dart';
/// @docImport 'package:flutter/material.dart';
library;

import 'package:flutter/foundation.dart';
import 'package:vector_math/vector_math_64.dart';

import 'layer.dart';
import 'object.dart';

/// The result after handling a [SelectionEvent].
///
/// [SelectionEvent]s are sent from [SelectionRegistrar] to be handled by
/// [SelectionHandler.dispatchSelectionEvent]. The subclasses of
/// [SelectionHandler] or [Selectable] must return appropriate
/// [SelectionResult]s after handling the events.
///
/// This is used by the [SelectionContainer] to determine how a selection
/// expands across its [Selectable] children.
enum SelectionResult {
  /// There is nothing left to select forward in this [Selectable], and further
  /// selection should extend to the next [Selectable] in screen order.
  ///
  /// {@template flutter.rendering.selection.SelectionResult.footNote}
  /// This is used after subclasses [SelectionHandler] or [Selectable] handled
  /// [SelectionEdgeUpdateEvent].
  /// {@endtemplate}
  next,

  /// Selection does not reach this [Selectable] and is located before it in
  /// screen order.
  ///
  /// {@macro flutter.rendering.selection.SelectionResult.footNote}
  previous,

  /// Selection ends in this [Selectable].
  ///
  /// Part of the [Selectable] may or may not be selected, but there is still
  /// content to select forward or backward.
  ///
  /// {@macro flutter.rendering.selection.SelectionResult.footNote}
  end,

  /// The result can't be determined in this frame.
  ///
  /// This is typically used when the subtree is scrolling to reveal more
  /// content.
  ///
  /// {@macro flutter.rendering.selection.SelectionResult.footNote}
  // See `_SelectableRegionState._triggerSelectionEndEdgeUpdate` for how this
  // result affects the selection.
  pending,

  /// There is no result for the selection event.
  ///
  /// This is used when a selection result is not applicable, e.g.
  /// [SelectAllSelectionEvent], [ClearSelectionEvent], and
  /// [SelectWordSelectionEvent].
  none,
}

/// The abstract interface to handle [SelectionEvent]s.
///
/// This interface is extended by [Selectable] and [SelectionContainerDelegate]
/// and is typically not used directly.
///
/// {@template flutter.rendering.SelectionHandler}
/// This class returns a [SelectionGeometry] as its [value], and is responsible
/// to notify its listener when its selection geometry has changed as the result
/// of receiving selection events.
/// {@endtemplate}
abstract class SelectionHandler implements ValueListenable<SelectionGeometry> {
  /// Marks this handler to be responsible for pushing [LeaderLayer]s for the
  /// selection handles.
  ///
  /// This handler is responsible for pushing the leader layers with the
  /// given layer links if they are not null. It is possible that only one layer
  /// is non-null if this handler is only responsible for pushing one layer
  /// link.
  ///
  /// The `startHandle` needs to be placed at the visual location of selection
  /// start, the `endHandle` needs to be placed at the visual location of selection
  /// end. Typically, the visual locations should be the same as
  /// [SelectionGeometry.startSelectionPoint] and
  /// [SelectionGeometry.endSelectionPoint].
  void pushHandleLayers(LayerLink? startHandle, LayerLink? endHandle);

  /// Gets the selected content in this object.
  ///
  /// Return `null` if nothing is selected.
  SelectedContent? getSelectedContent();

  /// Gets the [SelectedContentRange] representing the selected range in this object.
  ///
  /// When nothing is selected, subclasses should return `null`.
  SelectedContentRange? getSelection();

  /// Handles the [SelectionEvent] sent to this object.
  ///
  /// The subclasses need to update their selections or delegate the
  /// [SelectionEvent]s to their subtrees.
  ///
  /// The `event`s are subclasses of [SelectionEvent]. Check
  /// [SelectionEvent.type] to determine what kinds of event are dispatched to
  /// this handler and handle them accordingly.
  ///
  /// See also:
  ///  * [SelectionEventType], which contains all of the possible types.
  SelectionResult dispatchSelectionEvent(SelectionEvent event);

  /// The length of the content in this object.
  int get contentLength;
}

/// This class stores the range information of the selection under a [Selectable]
/// or [SelectionHandler].
///
/// The [SelectedContentRange] for a given [Selectable] or [SelectionHandler]
/// can be retrieved by calling [SelectionHandler.getSelection].
@immutable
class SelectedContentRange with Diagnosticable {
  /// Creates a [SelectedContentRange] with the given values.
  const SelectedContentRange({required this.startOffset, required this.endOffset})
    : assert((startOffset >= 0 && endOffset >= 0));

  /// The start of the selection relative to the start of the content.
  ///
  /// {@template flutter.rendering.selection.SelectedContentRange.selectionOffsets}
  /// For example a [Text] widget's content is in the format of an [TextSpan] tree.
  ///
  /// Take the [Text] widget and [TextSpan] tree below:
  ///
  /// {@tool snippet}
  /// ```dart
  /// const Text.rich(
  ///   TextSpan(
  ///     text: 'Hello world, ',
  ///     children: <InlineSpan>[
  ///       WidgetSpan(
  ///         child: Text('how are you today? '),
  ///       ),
  ///       TextSpan(
  ///         text: 'Good, thanks for asking.',
  ///       ),
  ///     ],
  ///   ),
  /// )
  /// ```
  /// {@end-tool}
  ///
  /// If we select from the beginning of 'world' to the end of the '.'
  /// at the end of the [TextSpan] tree, the [SelectedContentRange] from
  /// [SelectionHandler.getSelection] will be relative to the text of the
  /// [TextSpan] tree, with [WidgetSpan] content being flattened. The [startOffset]
  /// will be 6, and [endOffset] will be 56. This takes into account the
  /// length of the content in the [WidgetSpan], which is 19, making the overall
  /// length of the content 56.
  /// {@endtemplate}
  final int startOffset;

  /// The end of the selection relative to the start of the content.
  ///
  /// {@macro flutter.rendering.selection.SelectedContentRange.selectionOffsets}
  final int endOffset;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is SelectedContentRange &&
        other.startOffset == startOffset &&
        other.endOffset == endOffset;
  }

  @override
  int get hashCode {
    return Object.hash(startOffset, endOffset);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(IntProperty('startOffset', startOffset));
    properties.add(IntProperty('endOffset', endOffset));
  }
}

/// The selected content in a [Selectable] or [SelectionHandler].
// TODO(chunhtai): Add more support for rich content.
// https://github.com/flutter/flutter/issues/104206.
@immutable
class SelectedContent with Diagnosticable {
  /// Creates a selected content object.
  ///
  /// Only supports plain text.
  const SelectedContent({required this.plainText});

  /// The selected content in plain text format.
  final String plainText;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(StringProperty('plainText', plainText));
  }
}

/// A mixin that can be selected by users when under a [SelectionArea] widget.
///
/// This object receives selection events and the [value] must reflect the
/// current selection in this [Selectable]. The object must also notify its
/// listener if the [value] ever changes.
///
/// This object is responsible for drawing the selection highlight.
///
/// In order to receive the selection event, the mixer needs to register
/// itself to [SelectionRegistrar]s. Use
/// [SelectionContainer.maybeOf] to get the selection registrar, and
/// mix the [SelectionRegistrant] to subscribe to the [SelectionRegistrar]
/// automatically.
///
/// This mixin is typically mixed by [RenderObject]s. The [RenderObject.paint]
/// methods are responsible to push the [LayerLink]s provided to
/// [pushHandleLayers].
///
/// {@macro flutter.rendering.SelectionHandler}
///
/// See also:
///  * [SelectableRegion], which provides an overview of selection system.
mixin Selectable implements SelectionHandler {
  /// {@macro flutter.rendering.RenderObject.getTransformTo}
  Matrix4 getTransformTo(RenderObject? ancestor);

  /// The size of this [Selectable].
  Size get size;

  /// A list of [Rect]s that represent the bounding box of this [Selectable]
  /// in local coordinates.
  List<Rect> get boundingBoxes;

  /// Disposes resources held by the mixer.
  void dispose();
}

/// A mixin to auto-register the mixer to the [registrar].
///
/// To use this mixin, the mixer needs to set the [registrar] to the
/// [SelectionRegistrar] it wants to register to.
///
/// This mixin only registers the mixer with the [registrar] if the
/// [SelectionGeometry.hasContent] returned by the mixer is true.
mixin SelectionRegistrant on Selectable {
  /// The [SelectionRegistrar] the mixer will be or is registered to.
  ///
  /// This [Selectable] only registers the mixer if the
  /// [SelectionGeometry.hasContent] returned by the [Selectable] is true.
  SelectionRegistrar? get registrar => _registrar;
  SelectionRegistrar? _registrar;
  set registrar(SelectionRegistrar? value) {
    if (value == _registrar) {
      return;
    }
    if (value == null) {
      // When registrar goes from non-null to null;
      removeListener(_updateSelectionRegistrarSubscription);
    } else if (_registrar == null) {
      // When registrar goes from null to non-null;
      addListener(_updateSelectionRegistrarSubscription);
    }
    _removeSelectionRegistrarSubscription();
    _registrar = value;
    _updateSelectionRegistrarSubscription();
  }

  @override
  void dispose() {
    _removeSelectionRegistrarSubscription();
    super.dispose();
  }

  bool _subscribedToSelectionRegistrar = false;
  void _updateSelectionRegistrarSubscription() {
    if (_registrar == null) {
      _subscribedToSelectionRegistrar = false;
      return;
    }
    if (_subscribedToSelectionRegistrar && !value.hasContent) {
      _registrar!.remove(this);
      _subscribedToSelectionRegistrar = false;
    } else if (!_subscribedToSelectionRegistrar && value.hasContent) {
      _registrar!.add(this);
      _subscribedToSelectionRegistrar = true;
    }
  }

  void _removeSelectionRegistrarSubscription() {
    if (_subscribedToSelectionRegistrar) {
      _registrar!.remove(this);
      _subscribedToSelectionRegistrar = false;
    }
  }
}

/// A utility class that provides useful methods for handling selection events.
abstract final class SelectionUtils {
  /// Determines [SelectionResult] purely based on the target rectangle.
  ///
  /// This method returns [SelectionResult.end] if the `point` is inside the
  /// `targetRect`. Returns [SelectionResult.previous] if the `point` is
  /// considered to be lower than `targetRect` in screen order. Returns
  /// [SelectionResult.next] if the point is considered to be higher than
  /// `targetRect` in screen order.
  static SelectionResult getResultBasedOnRect(Rect targetRect, Offset point) {
    if (targetRect.contains(point)) {
      return SelectionResult.end;
    }
    if (point.dy < targetRect.top) {
      return SelectionResult.previous;
    }
    if (point.dy > targetRect.bottom) {
      return SelectionResult.next;
    }
    return point.dx >= targetRect.right ? SelectionResult.next : SelectionResult.previous;
  }

  /// Adjusts the dragging offset based on the target rect.
  ///
  /// This method moves the offsets to be within the target rect in case they are
  /// outside the rect.
  ///
  /// This is used in the case where a drag happens outside of the rectangle
  /// of a [Selectable].
  ///
  /// The logic works as the following:
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/rendering/adjust_drag_offset.png)
  ///
  /// For points inside the rect:
  ///   Their effective locations are unchanged.
  ///
  /// For points in Area 1:
  ///   Move them to top-left of the rect if text direction is ltr, or top-right
  ///   if rtl.
  ///
  /// For points in Area 2:
  ///   Move them to bottom-right of the rect if text direction is ltr, or
  ///   bottom-left if rtl.
  static Offset adjustDragOffset(
    Rect targetRect,
    Offset point, {
    TextDirection direction = TextDirection.ltr,
  }) {
    if (targetRect.contains(point)) {
      return point;
    }
    if (point.dy <= targetRect.top ||
        point.dy <= targetRect.bottom && point.dx <= targetRect.left) {
      // Area 1
      return direction == TextDirection.ltr ? targetRect.topLeft : targetRect.topRight;
    } else {
      // Area 2
      return direction == TextDirection.ltr ? targetRect.bottomRight : targetRect.bottomLeft;
    }
  }
}

/// The type of a [SelectionEvent].
///
/// Used by [SelectionEvent.type] to distinguish different types of events.
enum SelectionEventType {
  /// An event to update the selection start edge.
  ///
  /// Used by [SelectionEdgeUpdateEvent].
  startEdgeUpdate,

  /// An event to update the selection end edge.
  ///
  /// Used by [SelectionEdgeUpdateEvent].
  endEdgeUpdate,

  /// An event to clear the current selection.
  ///
  /// Used by [ClearSelectionEvent].
  clear,

  /// An event to select all the available content.
  ///
  /// Used by [SelectAllSelectionEvent].
  selectAll,

  /// An event to select a word at the location
  /// [SelectWordSelectionEvent.globalPosition].
  ///
  /// Used by [SelectWordSelectionEvent].
  selectWord,

  /// An event to select a paragraph at the location
  /// [SelectParagraphSelectionEvent.globalPosition].
  ///
  /// Used by [SelectParagraphSelectionEvent].
  selectParagraph,

  /// An event that extends the selection by a specific [TextGranularity].
  granularlyExtendSelection,

  /// An event that extends the selection in a specific direction.
  directionallyExtendSelection,
}

/// The unit of how selection handles move in text.
///
/// The [GranularlyExtendSelectionEvent] uses this enum to describe how
/// [Selectable] should extend its selection.
enum TextGranularity {
  /// Treats each character as an atomic unit when moving the selection handles.
  character,

  /// Treats word as an atomic unit when moving the selection handles.
  word,

  /// Treats a paragraph as an atomic unit when moving the selection handles.
  paragraph,

  /// Treats each line break as an atomic unit when moving the selection handles.
  line,

  /// Treats the entire document as an atomic unit when moving the selection handles.
  document,
}

/// An abstract base class for selection events.
///
/// This should not be directly used. To handle a selection event, it should
/// be downcast to a specific subclass. One can use [type] to look up which
/// subclasses to downcast to.
///
/// See also:
/// * [SelectAllSelectionEvent], for events to select all contents.
/// * [ClearSelectionEvent], for events to clear selections.
/// * [SelectWordSelectionEvent], for events to select words at the locations.
/// * [SelectionEdgeUpdateEvent], for events to update selection edges.
/// * [SelectionEventType], for determining the subclass types.
abstract class SelectionEvent {
  const SelectionEvent._(this.type);

  /// The type of this selection event.
  final SelectionEventType type;
}

/// Selects all selectable contents.
///
/// This event can be sent as the result of keyboard select-all, i.e.
/// ctrl + A, or cmd + A in macOS.
class SelectAllSelectionEvent extends SelectionEvent {
  /// Creates a select all selection event.
  const SelectAllSelectionEvent() : super._(SelectionEventType.selectAll);
}

/// Clears the selection from the [Selectable] and removes any existing
/// highlight as if there is no selection at all.
class ClearSelectionEvent extends SelectionEvent {
  /// Create a clear selection event.
  const ClearSelectionEvent() : super._(SelectionEventType.clear);
}

/// Selects the whole word at the location.
///
/// This event can be sent as the result of mobile long press selection.
class SelectWordSelectionEvent extends SelectionEvent {
  /// Creates a select word event at the [globalPosition].
  const SelectWordSelectionEvent({required this.globalPosition})
    : super._(SelectionEventType.selectWord);

  /// The position in global coordinates to select word at.
  final Offset globalPosition;
}

/// Selects the entire paragraph at the location.
///
/// This event can be sent as the result of a triple click to select.
class SelectParagraphSelectionEvent extends SelectionEvent {
  /// Creates a select paragraph event at the [globalPosition].
  const SelectParagraphSelectionEvent({required this.globalPosition, this.absorb = false})
    : super._(SelectionEventType.selectParagraph);

  /// The position in global coordinates to select paragraph at.
  final Offset globalPosition;

  /// Whether the selectable receiving the event should be absorbed into
  /// an encompassing paragraph.
  final bool absorb;
}

/// Updates a selection edge.
///
/// An active selection contains two edges, start and end. Use the [type] to
/// determine which edge this event applies to. If the [type] is
/// [SelectionEventType.startEdgeUpdate], the event updates start edge. If the
/// [type] is [SelectionEventType.endEdgeUpdate], the event updates end edge.
///
/// The [globalPosition] contains the new offset of the edge.
///
/// The [granularity] contains the granularity that the selection edge should move by.
/// Only [TextGranularity.character] and [TextGranularity.word] are currently supported.
///
/// This event is dispatched when the framework detects [TapDragStartDetails] in
/// [SelectionArea]'s gesture recognizers for mouse devices, or the selection
/// handles have been dragged to new locations.
class SelectionEdgeUpdateEvent extends SelectionEvent {
  /// Creates a selection start edge update event.
  ///
  /// The [globalPosition] contains the location of the selection start edge.
  ///
  /// The [granularity] contains the granularity which the selection edge should move by.
  /// This value defaults to [TextGranularity.character].
  const SelectionEdgeUpdateEvent.forStart({
    required this.globalPosition,
    TextGranularity? granularity,
  }) : granularity = granularity ?? TextGranularity.character,
       super._(SelectionEventType.startEdgeUpdate);

  /// Creates a selection end edge update event.
  ///
  /// The [globalPosition] contains the new location of the selection end edge.
  ///
  /// The [granularity] contains the granularity which the selection edge should move by.
  /// This value defaults to [TextGranularity.character].
  const SelectionEdgeUpdateEvent.forEnd({
    required this.globalPosition,
    TextGranularity? granularity,
  }) : granularity = granularity ?? TextGranularity.character,
       super._(SelectionEventType.endEdgeUpdate);

  /// The new location of the selection edge.
  final Offset globalPosition;

  /// The granularity for which the selection moves.
  ///
  /// Only [TextGranularity.character] and [TextGranularity.word] are currently supported.
  ///
  /// Defaults to [TextGranularity.character].
  final TextGranularity granularity;
}

/// Extends the start or end of the selection by a given [TextGranularity].
///
/// To handle this event, move the associated selection edge, as dictated by
/// [isEnd], according to the [granularity].
class GranularlyExtendSelectionEvent extends SelectionEvent {
  /// Creates a [GranularlyExtendSelectionEvent].
  const GranularlyExtendSelectionEvent({
    required this.forward,
    required this.isEnd,
    required this.granularity,
  }) : super._(SelectionEventType.granularlyExtendSelection);

  /// Whether to extend the selection forward.
  final bool forward;

  /// Whether this event is updating the end selection edge.
  final bool isEnd;

  /// The granularity for which the selection extend.
  final TextGranularity granularity;
}

/// The direction to extend a selection.
///
/// The [DirectionallyExtendSelectionEvent] uses this enum to describe how
/// [Selectable] should extend their selection.
enum SelectionExtendDirection {
  /// Move one edge of the selection vertically to the previous adjacent line.
  ///
  /// For text selection, it should consider both soft and hard linebreak.
  ///
  /// See [DirectionallyExtendSelectionEvent.dx] on how to
  /// calculate the horizontal offset.
  previousLine,

  /// Move one edge of the selection vertically to the next adjacent line.
  ///
  /// For text selection, it should consider both soft and hard linebreak.
  ///
  /// See [DirectionallyExtendSelectionEvent.dx] on how to
  /// calculate the horizontal offset.
  nextLine,

  /// Move the selection edges forward to a certain horizontal offset in the
  /// same line.
  ///
  /// If there is no on-going selection, the selection must start with the first
  /// line (or equivalence of first line in a non-text selectable) and select
  /// toward the horizontal offset in the same line.
  ///
  /// The selectable that receives [DirectionallyExtendSelectionEvent] with this
  /// enum must return [SelectionResult.end].
  ///
  /// See [DirectionallyExtendSelectionEvent.dx] on how to
  /// calculate the horizontal offset.
  forward,

  /// Move the selection edges backward to a certain horizontal offset in the
  /// same line.
  ///
  /// If there is no on-going selection, the selection must start with the last
  /// line (or equivalence of last line in a non-text selectable) and select
  /// backward the horizontal offset in the same line.
  ///
  /// The selectable that receives [DirectionallyExtendSelectionEvent] with this
  /// enum must return [SelectionResult.end].
  ///
  /// See [DirectionallyExtendSelectionEvent.dx] on how to
  /// calculate the horizontal offset.
  backward,
}

/// Extends the current selection with respect to a [direction].
///
/// To handle this event, move the associated selection edge, as dictated by
/// [isEnd], according to the [direction].
///
/// The movements are always based on [dx]. The value is in
/// global coordinates and is the horizontal offset the selection edge should
/// move to when moving to across lines.
class DirectionallyExtendSelectionEvent extends SelectionEvent {
  /// Creates a [DirectionallyExtendSelectionEvent].
  const DirectionallyExtendSelectionEvent({
    required this.dx,
    required this.isEnd,
    required this.direction,
  }) : super._(SelectionEventType.directionallyExtendSelection);

  /// The horizontal offset the selection should move to.
  ///
  /// The offset is in global coordinates.
  final double dx;

  /// Whether this event is updating the end selection edge.
  final bool isEnd;

  /// The directional movement of this event.
  ///
  /// See also:
  ///  * [SelectionExtendDirection], which explains how to handle each enum.
  final SelectionExtendDirection direction;

  /// Makes a copy of this object with its property replaced with the new
  /// values.
  DirectionallyExtendSelectionEvent copyWith({
    double? dx,
    bool? isEnd,
    SelectionExtendDirection? direction,
  }) {
    return DirectionallyExtendSelectionEvent(
      dx: dx ?? this.dx,
      isEnd: isEnd ?? this.isEnd,
      direction: direction ?? this.direction,
    );
  }
}

/// A registrar that keeps track of [Selectable]s in the subtree.
///
/// A [Selectable] is only included in the [SelectableRegion] if they are
/// registered with a [SelectionRegistrar]. Once a [Selectable] is registered,
/// it will receive [SelectionEvent]s in
/// [SelectionHandler.dispatchSelectionEvent].
///
/// Use [SelectionContainer.maybeOf] to get the immediate [SelectionRegistrar]
/// in the ancestor chain above the build context.
///
/// See also:
///  * [SelectableRegion], which provides an overview of the selection system.
///  * [SelectionRegistrarScope], which hosts the [SelectionRegistrar] for the
///    subtree.
///  * [SelectionRegistrant], which auto registers the object with the mixin to
///    [SelectionRegistrar].
abstract class SelectionRegistrar {
  /// Adds the [selectable] into the registrar.
  ///
  /// A [Selectable] must register with the [SelectionRegistrar] in order to
  /// receive selection events.
  void add(Selectable selectable);

  /// Removes the [selectable] from the registrar.
  ///
  /// A [Selectable] must unregister itself if it is removed from the rendering
  /// tree.
  void remove(Selectable selectable);
}

/// The status that indicates whether there is a selection and whether the
/// selection is collapsed.
///
/// A collapsed selection means the selection starts and ends at the same
/// location.
enum SelectionStatus {
  /// The selection is not collapsed.
  ///
  /// For example if `{}` represent the selection edges:
  ///   'ab{cd}', the collapsing status is [uncollapsed].
  ///   '{abcd}', the collapsing status is [uncollapsed].
  uncollapsed,

  /// The selection is collapsed.
  ///
  /// For example if `{}` represent the selection edges:
  ///   'ab{}cd', the collapsing status is [collapsed].
  ///   '{}abcd', the collapsing status is [collapsed].
  ///   'abcd{}', the collapsing status is [collapsed].
  collapsed,

  /// No selection.
  none,
}

/// The geometry of the current selection.
///
/// This includes details such as the locations of the selection start and end,
/// line height, the rects that encompass the selection, etc. This information
/// is used for drawing selection controls for mobile platforms.
///
/// The positions in geometry are in local coordinates of the [SelectionHandler]
/// or [Selectable].
@immutable
class SelectionGeometry with Diagnosticable {
  /// Creates a selection geometry object.
  ///
  /// If any of the [startSelectionPoint] and [endSelectionPoint] is not null,
  /// the [status] must not be [SelectionStatus.none].
  const SelectionGeometry({
    this.startSelectionPoint,
    this.endSelectionPoint,
    this.selectionRects = const <Rect>[],
    required this.status,
    required this.hasContent,
  }) : assert(
         (startSelectionPoint == null && endSelectionPoint == null) ||
             status != SelectionStatus.none,
       );

  /// The geometry information at the selection start.
  ///
  /// This information is used for drawing mobile selection controls. The
  /// [SelectionPoint.localPosition] of the selection start is typically at the
  /// start of the selection highlight at where the start selection handle
  /// should be drawn.
  ///
  /// The [SelectionPoint.handleType] should be [TextSelectionHandleType.left]
  /// for forward selection or [TextSelectionHandleType.right] for backward
  /// selection in most cases.
  ///
  /// Can be null if the selection start is offstage, for example, when the
  /// selection is outside of the viewport or is kept alive by a scrollable.
  final SelectionPoint? startSelectionPoint;

  /// The geometry information at the selection end.
  ///
  /// This information is used for drawing mobile selection controls. The
  /// [SelectionPoint.localPosition] of the selection end is typically at the end
  /// of the selection highlight at where the end selection handle should be
  /// drawn.
  ///
  /// The [SelectionPoint.handleType] should be [TextSelectionHandleType.right]
  /// for forward selection or [TextSelectionHandleType.left] for backward
  /// selection in most cases.
  ///
  /// Can be null if the selection end is offstage, for example, when the
  /// selection is outside of the viewport or is kept alive by a scrollable.
  final SelectionPoint? endSelectionPoint;

  /// The status of ongoing selection in the [Selectable] or [SelectionHandler].
  final SelectionStatus status;

  /// The rects in the local coordinates of the containing [Selectable] that
  /// represent the selection if there is any.
  final List<Rect> selectionRects;

  /// Whether there is any selectable content in the [Selectable] or
  /// [SelectionHandler].
  final bool hasContent;

  /// Whether there is an ongoing selection.
  bool get hasSelection => status != SelectionStatus.none;

  /// Makes a copy of this object with the given values updated.
  SelectionGeometry copyWith({
    SelectionPoint? startSelectionPoint,
    SelectionPoint? endSelectionPoint,
    List<Rect>? selectionRects,
    SelectionStatus? status,
    bool? hasContent,
  }) {
    return SelectionGeometry(
      startSelectionPoint: startSelectionPoint ?? this.startSelectionPoint,
      endSelectionPoint: endSelectionPoint ?? this.endSelectionPoint,
      selectionRects: selectionRects ?? this.selectionRects,
      status: status ?? this.status,
      hasContent: hasContent ?? this.hasContent,
    );
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is SelectionGeometry &&
        other.startSelectionPoint == startSelectionPoint &&
        other.endSelectionPoint == endSelectionPoint &&
        listEquals(other.selectionRects, selectionRects) &&
        other.status == status &&
        other.hasContent == hasContent;
  }

  @override
  int get hashCode {
    return Object.hash(startSelectionPoint, endSelectionPoint, selectionRects, status, hasContent);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<SelectionPoint>('startSelectionPoint', startSelectionPoint));
    properties.add(DiagnosticsProperty<SelectionPoint>('endSelectionPoint', endSelectionPoint));
    properties.add(IterableProperty<Rect>('selectionRects', selectionRects));
    properties.add(EnumProperty<SelectionStatus>('status', status));
    properties.add(DiagnosticsProperty<bool>('hasContent', hasContent));
  }
}

/// The geometry information of a selection point.
@immutable
class SelectionPoint with Diagnosticable {
  /// Creates a selection point object.
  const SelectionPoint({
    required this.localPosition,
    required this.lineHeight,
    required this.handleType,
  });

  /// The position of the selection point in the local coordinates of the
  /// containing [Selectable].
  final Offset localPosition;

  /// The line height at the selection point.
  final double lineHeight;

  /// The selection handle type that should be used at the selection point.
  ///
  /// This is used for building the mobile selection handle.
  final TextSelectionHandleType handleType;

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is SelectionPoint &&
        other.localPosition == localPosition &&
        other.lineHeight == lineHeight &&
        other.handleType == handleType;
  }

  @override
  int get hashCode {
    return Object.hash(localPosition, lineHeight, handleType);
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<Offset>('localPosition', localPosition));
    properties.add(DoubleProperty('lineHeight', lineHeight));
    properties.add(EnumProperty<TextSelectionHandleType>('handleType', handleType));
  }
}

/// The type of selection handle to be displayed.
///
/// With mixed-direction text, both handles may be the same type. Examples:
///
/// * LTR text: 'the &lt;quick brown&gt; fox':
///
///   The '&lt;' is drawn with the [left] type, the '&gt;' with the [right]
///
/// * RTL text: 'XOF &lt;NWORB KCIUQ&gt; EHT':
///
///   Same as above.
///
/// * mixed text: '&lt;the NWOR&lt;B KCIUQ fox'
///
///   Here 'the QUICK B' is selected, but 'QUICK BROWN' is RTL. Both are drawn
///   with the [left] type.
///
/// See also:
///
///  * [TextDirection], which discusses left-to-right and right-to-left text in
///    more detail.
enum TextSelectionHandleType {
  /// The selection handle is to the left of the selection end point.
  left,

  /// The selection handle is to the right of the selection end point.
  right,

  /// The start and end of the selection are co-incident at this point.
  collapsed,
}
